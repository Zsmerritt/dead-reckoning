"""Client link from the plugin to the plrd recovery daemon.

Control-socket protocol (FIXED — the Rust daemon implements the server
side against exactly this framing):

* transport: UNIX stream socket at the [plr] ``control_socket`` path;
* request: one line of JSON, newline-terminated:
  ``{"cmd": "<name>", "args": {...}}\\n``;
* response: one line of JSON, newline-terminated:
  ``{"ok": bool, "text": "<human-readable report>", "data": {...}}\\n``;
* commands: ``ping``, ``status``, ``recover_dryrun``,
  ``recover_execute`` (args ``{"confirm": true, "step": bool}``).

The client keeps timeouts short and errors human: plrd is Linux-only
and may simply not be running, and every failure surfaces as a clear
console message instead of a traceback.  Socket creation is injectable
(``connect_factory``) so the framing code is exercised by tests over
real sockets on every platform, while the AF_UNIX default path is
covered where the OS provides it.

The two PLR commands this module backs raise errors through
``gcmd.error`` (klippy/gcode.py:24-25: GCodeCommand.error is
CommandError, which klippy reports on the console).
"""

import json
import socket

# Timeouts per call type (seconds).  ping/status are console-latency
# bound; recover_* waits for plrd's full validate/plan/execute cycle.
PING_TIMEOUT = 1.0
STATUS_TIMEOUT = 5.0
RECOVER_TIMEOUT = 120.0

MIN_TIMEOUT = 0.05
MAX_TIMEOUT = 120.0

# Hard cap on a single response line.  A response beyond this indicates
# a broken or hostile server, not a long report.
MAX_RESPONSE_BYTES = 4 * 1024 * 1024


class DaemonError(Exception):
    """A daemon call failed; str() is a console-ready message."""


def validate_timeout(seconds, minimum=MIN_TIMEOUT, maximum=MAX_TIMEOUT):
    """Validate a daemon-call timeout in seconds; return it as float.

    Raises ValueError outside [minimum, maximum]: a too-small timeout
    fails spuriously, and an over-long one would stall the klippy
    reactor from a console command.
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
        """True if plrd answers a ping in time; never raises."""
        try:
            return bool(self.call("ping", timeout=PING_TIMEOUT)["ok"])
        except DaemonError:
            return False


def cmd_PLR_STATUS(plugin, gcmd):
    """PLR_STATUS — plugin-side state plus the daemon's status report."""
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
    lines.append("plrd (%s):" % (plugin.control_socket,))
    try:
        resp = plugin.daemon.call("status", timeout=STATUS_TIMEOUT)
    except DaemonError as e:
        lines.append("  %s" % (e,))
    else:
        text = resp["text"] or ("ok" if resp["ok"] else "daemon reported failure")
        for text_line in text.splitlines() or [text]:
            lines.append("  %s" % (text_line,))
    gcmd.respond_info("\n".join(lines))


def cmd_PLR_RECOVER(plugin, gcmd):
    """PLR_RECOVER [EXECUTE=1 CONFIRM=YES] [STEP=1] — recovery via plrd.

    Default is a dry run: plrd validates the machine and prints the
    plan without motion.  Execution demands the exact argument pair
    EXECUTE=1 CONFIRM=YES — anything less refuses client-side.  plrd
    still enforces every gate server-side (machine validation,
    klippy-ready+idle, transcript), so this consent gate is additive,
    not the safety mechanism.
    """
    execute = gcmd.get_int("EXECUTE", 0)
    step = bool(gcmd.get_int("STEP", 0))
    if not execute:
        resp = _call_or_error(plugin, gcmd, "recover_dryrun", None)
        gcmd.respond_info(resp["text"] or "plrd returned an empty dry-run report")
        if not resp["ok"]:
            raise gcmd.error("plrd dry run reported failure (see report above)")
        return
    confirm = gcmd.get("CONFIRM", "")
    if confirm != "YES":
        raise gcmd.error(
            "PLR_RECOVER EXECUTE=1 moves the machine. Refusing without "
            "CONFIRM=YES (literally). Run PLR_RECOVER first for a dry run."
        )
    resp = _call_or_error(
        plugin, gcmd, "recover_execute", {"confirm": True, "step": step}
    )
    gcmd.respond_info(resp["text"] or "plrd returned an empty recovery report")
    if not resp["ok"]:
        raise gcmd.error("plrd recovery reported failure (see report above)")


def _call_or_error(plugin, gcmd, cmd, args):
    try:
        return plugin.daemon.call(cmd, args, timeout=RECOVER_TIMEOUT)
    except DaemonError as e:
        raise gcmd.error(str(e)) from None
