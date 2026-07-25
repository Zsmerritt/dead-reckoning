"""A real plrd control-socket server for the reactor harness.

Speaks the protocol verbatim (crates/plrd/src/ctrlsock.rs:5-23): one
newline-terminated JSON request per connection, one newline-terminated
JSON response, then close.  Only the SERVER is fake — the client under
test is the shipped :class:`plr.daemon_link.DaemonLink`, over real
sockets, from real worker threads.

AF_INET on the loopback rather than AF_UNIX so the harness runs on every
host the suite runs on (CPython on Windows has no ``AF_UNIX``, which is
why two framing tests in tests/test_daemon_link.py skip there).  The
framing is transport-independent, and ``DaemonLink``'s injectable
``connect_factory`` is the seam the production code already provides.

The two knobs that make the reactor proof possible:

* ``delay`` — how long the server takes to answer, i.e. exactly the thing
  the plugin used to spend inside a g-code handler;
* ``on_request`` — called on the server thread when a request arrives,
  used to make plrd do what a real recovery does at that moment: push
  g-code at klippy and wait for it.  That is the other half of the
  deadlock (crates/plrd/src/moonraker.rs:167-172 →
  klippy/webhooks.py:447-448 → the g-code mutex).
"""

import json
import socket
import threading

from plr import daemon_link


class FakePlrd:
    """A threaded, loopback plrd control socket with scripted responses."""

    def __init__(self, script=None, delay=0.0, on_request=None, hang=False):
        self.script = {cmd: list(v) for cmd, v in (script or {}).items()}
        self.delay = delay
        self.on_request = on_request
        self.hang = hang
        self.requests = []
        self._open = []
        self._lock = threading.Lock()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(8)
        self.address = self._listener.getsockname()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, name="fake-plrd")
        self._thread.daemon = True
        self._thread.start()

    # -- lifecycle ----------------------------------------------------

    def close(self):
        """Release every waiting client, then stop listening.

        Hung connections are ANSWERED on the way out rather than dropped, so
        the plugin's worker threads unblock immediately and a test can join
        them.  Otherwise a worker sitting on a 3600-second recovery deadline
        outlives the test and reports into a torn-down harness at
        interpreter exit — which is how a leaked traceback appears after the
        summary line.
        """
        self._stop.set()
        with self._lock:
            waiting, self._open = self._open, []
        for conn in waiting:
            try:
                conn.sendall(
                    (
                        json.dumps(
                            {
                                "ok": False,
                                "text": "fake plrd shutting down",
                                "data": {"outcome": "error"},
                            }
                        )
                        + "\n"
                    ).encode("utf-8")
                )
            except OSError:
                pass
        try:
            self._listener.close()
        except OSError:
            pass
        # The released connections are left for their handler threads to
        # close, so the client gets a chance to read the line first.

    def connect_factory(self):
        """A ``DaemonLink`` connect_factory pointing at this server.

        Mirrors the production ``_default_connect`` (plr/daemon_link.py) in
        the one behaviour that matters here: a failed connect is a
        ``DaemonError``, not a bare ``OSError``, so the plugin's error path
        is the one under test rather than its unexpected-exception path.
        """

        def connect(path, timeout):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            try:
                sock.connect(self.address)
            except OSError:
                sock.close()
                raise daemon_link.DaemonError(
                    "plrd not reachable at %s (fake)" % (self.address,)
                ) from None
            return sock

        return connect

    # -- server -------------------------------------------------------

    def _serve(self):
        while not self._stop.is_set():
            try:
                conn, _addr = self._listener.accept()
            except OSError:
                return
            handler = threading.Thread(target=self._handle, args=(conn,))
            handler.daemon = True
            handler.start()

    def _handle(self, conn):
        try:
            buf = b""
            while b"\n" not in buf:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                buf += chunk
            request = json.loads(buf.split(b"\n", 1)[0].decode("utf-8"))
            cmd = request.get("cmd")
            args = request.get("args") or {}
            with self._lock:
                self.requests.append((cmd, args))
            if self.on_request is not None:
                # Runs on the server thread, exactly where plrd's own
                # Moonraker traffic happens: DURING the request it has not
                # answered yet.
                self.on_request(cmd, args)
            if self.hang:
                # Alive, connected, never answering: the client's own
                # deadline is the only exit — until `close` releases it.
                with self._lock:
                    self._open.append(conn)
                self._stop.wait()
                return
            if self.delay:
                self._stop.wait(self.delay)
            conn.sendall((json.dumps(self._response(cmd)) + "\n").encode("utf-8"))
        except (OSError, ValueError):
            # The client timed out and closed, or sent garbage: that is the
            # scenario under test, not a server failure.
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _response(self, cmd):
        with self._lock:
            queue = self.script.get(cmd)
            if not queue:
                return {"ok": True, "text": "", "data": {}}
            return queue.pop(0) if len(queue) > 1 else queue[0]
