"""Client link from the plugin to the plrd recovery daemon.

Control-socket protocol (FIXED — the Rust daemon implements the server
side against exactly this framing; crates/plrd/src/ctrlsock.rs:5-23):

* transport: UNIX stream socket at the [plr] ``control_socket`` path;
* request: one line of JSON, newline-terminated:
  ``{"cmd": "<name>", "args": {...}}\\n``;
* response: one line of JSON, newline-terminated:
  ``{"ok": bool, "text": "<human-readable report>", "data": {...}}\\n``;
* ONE request per connection: plrd writes the response and closes;
* commands: ``ping``, ``status``, ``recover_dryrun``, ``recover_execute``
  (args ``{"confirm": true, "on_confirm": "abort"|"ask"}``),
  ``recover_confirm`` (args ``{"token": str, "answer":
  "continue"|"abort"}``).

============================================================================
EVERY CALL IN THIS MODULE BLOCKS, AND MUST THEREFORE RUN ON A WORKER
THREAD (plr/daemon_worker.py) — NEVER INSIDE A G-CODE HANDLER.
============================================================================

``call`` does a blocking ``connect``/``sendall``/``recv``.  Klippy's
reactor is one thread that dispatches g-code handlers inline
(klippy/reactor.py:314-327), so a blocking call in a handler stalls
every timer and fd in klippy — which switches the heaters off within
``MAX_MAINTHREAD_TIME`` = 5 s (klippy/extras/heaters.py:17, :72-74,
:138-141) and can fault the printer into shutdown
(klippy/extras/verify_heater.py:86-91), while a heater PWM pin left
un-refreshed past ``MAX_HEAT_TIME`` = 3 s is an MCU-side shutdown by
construction (heaters.py:14, :62; src/pwmcmds.c:45-53).  The full
argument, including why plrd cannot make progress either, is in
plr/daemon_worker.py's module docstring.  Consequence for this file: no
timeout here is "short enough" to be safe on the reactor, and none of
these functions may be called from one.

The client's errors are human: plrd is Linux-only and may simply not be
running, and every failure surfaces as a clear console message instead of
a traceback.  Socket creation is injectable (``connect_factory``) so the
framing code is exercised by tests over real sockets on every platform,
while the AF_UNIX default path is covered where the OS provides it.

The PLR commands this module backs report errors through ``gcmd.error``
(klippy/gcode.py:24-25: GCodeCommand.error is CommandError, which klippy
reports on the console) for anything decided synchronously, and through
``gcode.respond_info`` for anything a worker learns later — a callback has
no ``gcmd`` to raise on, and an exception escaping a reactor callback is a
printer shutdown (klippy/klippy.py:170-186).
"""

import json
import socket

# Timeouts per call type (seconds).  These are spent on worker threads, so
# they are sized for what plrd legitimately takes, not for what a console
# can tolerate:
#
# * ping/status answer off local state and are console-latency bound;
# * recover_dryrun runs the whole WAL scan -> reconstruct -> plan pipeline
#   on a blocking thread pool (ctrlsock.rs:517-568) over a journal that can
#   be hundreds of megabytes;
# * recover_execute (and each recover_confirm that resumes it) returns only
#   when the recovery PAUSES or FINISHES (ctrlsock.rs:790-836), and plrd's
#   own step deadlines are minutes each — ``temp_timeout`` alone is 15 min
#   (crates/plrd/src/executor.rs:152) and a stock plan waits for the bed
#   and then the probe temperature.  This deadline must therefore outlast
#   plrd's own worst case; it exists only so a wedged daemon eventually
#   releases the plugin's single-flight slot, and it is never compared
#   against any plrd limit.
PING_TIMEOUT = 1.0
STATUS_TIMEOUT = 5.0
DRYRUN_TIMEOUT = 300.0
EXECUTE_TIMEOUT = 3600.0

MIN_TIMEOUT = 0.05
# Sanity ceiling only.  It used to be 120 s to stop a console command
# stalling the reactor; that protection now comes from the call not being
# on the reactor at all, so the ceiling is sized for the longest legitimate
# conversation (a recovery, above) with room to spare.
MAX_TIMEOUT = 4 * 3600.0

# Hard cap on a single response line.  A response beyond this indicates
# a broken or hostile server, not a long report.
MAX_RESPONSE_BYTES = 4 * 1024 * 1024


class DaemonError(Exception):
    """A daemon call failed; str() is a console-ready message."""


def validate_timeout(seconds, minimum=MIN_TIMEOUT, maximum=MAX_TIMEOUT):
    """Validate a daemon-call timeout in seconds; return it as float.

    Raises ValueError outside [minimum, maximum]: a too-small timeout
    fails spuriously, and an unbounded one would hold a worker thread —
    and with it this plugin's single-flight slot — forever against a
    daemon that is alive but never answers.
    """
    seconds = float(seconds)
    if seconds < minimum or seconds > maximum:
        raise ValueError(
            "timeout %.3fs out of range [%.3fs, %.3fs]" % (seconds, minimum, maximum)
        )
    return seconds


def _default_connect(path, timeout):
    """Open the AF_UNIX stream socket to plrd.

    AF_UNIX is absent from python builds on some platforms (notably
    CPython on Windows); that is a config/environment error surfaced as
    DaemonError, not a crash.
    """
    if not hasattr(socket, "AF_UNIX"):
        raise DaemonError(
            "this python build has no AF_UNIX support; plrd control "
            "sockets require a POSIX host"
        )
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(path)
    except OSError:
        sock.close()
        raise DaemonError(
            "plrd not reachable at %s — is the service running? "
            "(systemctl status plrd)" % (path,)
        ) from None
    return sock


class DaemonLink:
    """One-shot request/response client for the plrd control socket."""

    def __init__(self, socket_path, connect_factory=None):
        self.socket_path = socket_path
        self._connect = connect_factory or _default_connect

    def call(self, cmd, args=None, timeout=STATUS_TIMEOUT):
        """Send one request; return the response dict.

        Returns ``{"ok": bool, "text": str, "data": dict}`` after
        validating the frame.  Raises DaemonError on connect failure,
        timeout, oversized/truncated response, or malformed JSON.
        """
        timeout = validate_timeout(timeout)
        request = json.dumps({"cmd": cmd, "args": args or {}}) + "\n"
        sock = self._connect(self.socket_path, timeout)
        try:
            sock.sendall(request.encode("utf-8"))
            line = self._read_line(sock, cmd)
        except socket.timeout:
            raise DaemonError(
                "plrd did not answer '%s' within %.0fs at %s"
                % (cmd, timeout, self.socket_path)
            ) from None
        except OSError as e:
            raise DaemonError(
                "socket error talking to plrd at %s: %s" % (self.socket_path, e)
            ) from None
        finally:
            sock.close()
        return self._parse_response(line, cmd)

    def _read_line(self, sock, cmd):
        chunks = []
        total = 0
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                raise DaemonError(
                    "plrd closed the connection mid-response to '%s' at %s"
                    % (cmd, self.socket_path)
                )
            total += len(chunk)
            if total > MAX_RESPONSE_BYTES:
                raise DaemonError(
                    "plrd response to '%s' exceeded %d bytes — protocol "
                    "violation" % (cmd, MAX_RESPONSE_BYTES)
                )
            chunks.append(chunk)
            if b"\n" in chunk:
                break
        return b"".join(chunks).split(b"\n", 1)[0]

    def _parse_response(self, line, cmd):
        try:
            resp = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            raise DaemonError(
                "plrd sent a malformed response to '%s' (not JSON): %.80r" % (cmd, line)
            ) from None
        if not isinstance(resp, dict) or not isinstance(resp.get("ok"), bool):
            raise DaemonError(
                "plrd response to '%s' missing boolean 'ok' field: %.80r" % (cmd, line)
            )
        text = resp.get("text", "")
        if not isinstance(text, str):
            raise DaemonError(
                "plrd response to '%s' has non-string 'text' field" % (cmd,)
            )
        data = resp.get("data", {})
        if not isinstance(data, dict):
            raise DaemonError(
                "plrd response to '%s' has non-object 'data' field" % (cmd,)
            )
        return {"ok": resp["ok"], "text": text, "data": data}

    def ping(self):
        """True if plrd answers a ping in time; never raises.

        BLOCKS: worker threads only, like every other call here.  No
        production caller exists today — ``get_status``'s ``daemon_alive``
        deliberately uses the heartbeat file's mtime instead
        (plr/plugin.py ``_daemon_alive_now``) precisely because it runs on
        the reactor several times a second.
        """
        try:
            return bool(self.call("ping", timeout=PING_TIMEOUT)["ok"])
        except DaemonError:
            return False


def respond_error(gcode, msg):
    """Console error line(s) from a context with no ``gcmd`` to raise on.

    Mirrors klippy's own ``GCodeDispatch._respond_error``
    (klippy/gcode.py:257-263): the full text as ``//`` info lines, then
    the first line prefixed ``!!`` — the error prefix
    (klippy/extras/respond.py:8-12) clients render as a failure.  Used
    from reactor callbacks, where raising is not an option: an exception
    escaping one is a printer shutdown (klippy/klippy.py:170-186).
    """
    lines = str(msg).strip().split("\n")
    if len(lines) > 1:
        gcode.respond_info("\n".join(lines), log=False)
    gcode.respond_raw("!! %s" % (lines[0].strip(),))


def cmd_PLR_STATUS(plugin, gcmd):
    """PLR_STATUS — plugin-side state plus the daemon's status report.

    The plugin's own state prints immediately from the handler; the
    daemon's block arrives from a worker thread, because a blocking
    ``status`` call here would stall the reactor (module docstring) — with
    a 5 s timeout that already exceeded klippy's 3 s heater watchdog, on
    ANY printer, mid-print, with nothing to do with recovery.
    """
    lines = ["PLR plugin:"]
    lines.append("  probe_method: %s" % (plugin.probe_method,))
    if plugin.accel_chip_name:
        lines.append("  accel_chip: %s" % (plugin.accel_chip_name,))
    lines.append(
        "  self_locking_z attested: %s" % ("yes" if plugin.self_locking_z else "no")
    )
    if plugin.probe_resolution is not None:
        lines.append("  probe_resolution: %.6f mm" % (plugin.probe_resolution,))
    else:
        lines.append("  probe_resolution: not measured (run PLR_PROBE_TEST)")
    for name, value in plugin.tunables.items():
        pending = " [awaiting SAVE_CONFIG]" if plugin.is_pending_save(name) else ""
        lines.append("  %s: %g%s" % (name, value, pending))
    lines.extend("  %s" % (line,) for line in plugin.recovery.status_lines())
    gcode = plugin.printer.lookup_object("gcode")
    socket_path = plugin.control_socket

    def on_result(resp):
        text = resp["text"] or ("ok" if resp["ok"] else "daemon reported failure")
        gcode.respond_info(
            "plrd (%s):\n%s"
            % (socket_path, "\n".join("  %s" % (line,) for line in text.splitlines()))
        )

    def on_error(err):
        respond_error(gcode, "plrd (%s): %s" % (socket_path, err))

    lines.append("plrd (%s):" % (socket_path,))
    # Start the query BEFORE announcing it, so the announcement is true:
    # either a report is coming, or the console says why it is not.
    if plugin.daemon_query.call("status", None, STATUS_TIMEOUT, on_result, on_error):
        lines.append("  asking the daemon (its report follows)...")
    else:
        lines.append(
            "  a plrd query is already in flight; its report will appear "
            "when it answers."
        )
    gcmd.respond_info("\n".join(lines))


def cmd_PLR_RECOVER(plugin, gcmd):
    """PLR_RECOVER [EXECUTE=1 CONFIRM=YES] [STEP=1] — recovery via plrd.

    Default is a dry run: plrd validates the machine and prints the plan
    without motion.  Execution demands the exact argument pair EXECUTE=1
    CONFIRM=YES — anything less refuses client-side.  plrd still enforces
    every gate server-side (machine validation, klippy-ready+idle,
    transcript), so this consent gate is additive, not the safety
    mechanism.

    BOTH paths return immediately and report from worker threads
    (module docstring).  Execution then runs plrd's confirm-point loop
    exactly as the wizard does — they share one
    :class:`plr.recovery.RecoverySession`, so a second attempt from
    either entry point is refused by the same guard.
    """
    execute = gcmd.get_int("EXECUTE", 0)
    step = bool(gcmd.get_int("STEP", 0))
    if step:
        # ctrlsock.rs:603-605 answers `"step": true` with
        # `refused: per-step mode is CLI-only`, so sending it could only
        # ever produce a refusal.  Pausing before every step over the
        # socket is a [plr] key, and it lands in the confirm-point loop as
        # `step-debug` pauses.
        already = plugin.daemon_keys.get("debug_confirm_each_step") is True
        raise gcmd.error(
            "PLR_RECOVER STEP=1 is not how per-step confirmation works over "
            "the control socket — plrd refuses the argument outright. Set "
            "`debug_confirm_each_step: True` in printer.cfg's [plr] section "
            "and restart klippy; recovery then stops before EVERY step and "
            "asks. (%s)"
            % (
                "it is already set, so just run the recovery"
                if already
                else "it is not set at the moment"
            )
        )
    if not execute:
        gcmd.respond_info(
            "PLR_RECOVER: asking plrd for a dry run (no motion). This can take "
            "a while on a large journal; the plan appears here when it is ready."
        )
        gcode = plugin.printer.lookup_object("gcode")

        def on_result(resp):
            gcode.respond_info(resp["text"] or "plrd returned an empty dry-run report")
            if not resp["ok"]:
                respond_error(gcode, "plrd dry run reported failure (see report above)")

        def on_error(err):
            respond_error(gcode, "PLR_RECOVER dry run: %s" % (err,))

        if not plugin.daemon_query.call(
            "recover_dryrun", None, DRYRUN_TIMEOUT, on_result, on_error
        ):
            raise gcmd.error(
                "PLR_RECOVER: a plrd query is already in flight; wait for it to report."
            )
        return
    confirm = gcmd.get("CONFIRM", "")
    if confirm != "YES":
        raise gcmd.error(
            "PLR_RECOVER EXECUTE=1 moves the machine. Refusing without "
            "CONFIRM=YES (literally). Run PLR_RECOVER first for a dry run."
        )
    plugin.recovery.start(gcmd, "PLR_RECOVER")
