"""Tests for the plrd control-socket client and its console commands.

The protocol framing under test is the real shipped code path: tests
drive DaemonLink.call over REAL stream sockets (socketpair on every
platform; true AF_UNIX sockets where the OS provides them) against a
tiny in-test server that sends canned bytes.  Only the *server* is
fake — never the framing.
"""

import json
import os
import socket
import threading
import time

import fake_klippy
import pytest

from plr import daemon_link

HAS_AF_UNIX = hasattr(socket, "AF_UNIX")


def canned_link(server_bytes, request_log=None, close_early=False, delay=0.0):
    """DaemonLink over a real socketpair; the far end is a canned server."""

    def serve(server):
        try:
            buf = b""
            while b"\n" not in buf:
                chunk = server.recv(4096)
                if not chunk:
                    return
                buf += chunk
            if request_log is not None:
                request_log.append(buf)
            if delay:
                time.sleep(delay)
            if not close_early:
                server.sendall(server_bytes)
        except OSError:
            # The client side may have timed out and closed already;
            # that is the scenario under test, not a server failure.
            pass
        finally:
            server.close()

    def connect(path, timeout):
        client, server = socket.socketpair()
        client.settimeout(timeout)
        thread = threading.Thread(target=serve, args=(server,))
        thread.daemon = True
        thread.start()
        return client

    return daemon_link.DaemonLink("/run/fake/plrd.sock", connect_factory=connect)


def ok_bytes(text="all good", data=None):
    return (json.dumps({"ok": True, "text": text, "data": data or {}}) + "\n").encode(
        "utf-8"
    )


# --- timeout validation ------------------------------------------------


def test_valid_timeout_returned_as_float():
    result = daemon_link.validate_timeout(2)
    assert result == 2.0
    assert isinstance(result, float)


def test_bounds_are_inclusive():
    assert daemon_link.validate_timeout(daemon_link.MIN_TIMEOUT) == (
        daemon_link.MIN_TIMEOUT
    )
    assert daemon_link.validate_timeout(daemon_link.MAX_TIMEOUT) == (
        daemon_link.MAX_TIMEOUT
    )


@pytest.mark.parametrize("seconds", [0.0, 0.01, 14401.0, -5.0])
def test_out_of_range_rejected(seconds):
    with pytest.raises(ValueError, match="out of range"):
        daemon_link.validate_timeout(seconds)


def test_every_call_timeout_is_within_bounds():
    # Every per-command deadline must be a value `call` will accept: an
    # out-of-band constant would turn a daemon call into a ValueError from
    # a worker thread, which is the one place a raise helps nobody.
    for timeout in (
        daemon_link.PING_TIMEOUT,
        daemon_link.STATUS_TIMEOUT,
        daemon_link.DRYRUN_TIMEOUT,
        daemon_link.EXECUTE_TIMEOUT,
    ):
        assert daemon_link.validate_timeout(timeout) == timeout


def test_execute_timeout_outlasts_the_daemons_own_step_deadlines():
    # plrd answers recover_execute only when the recovery pauses or
    # finishes, and its own per-step deadlines are minutes each
    # (crates/plrd/src/executor.rs:150-155: verify_timeout 10 s,
    # temp_timeout 15 min).  A stock plan waits for the bed AND then the
    # probe temperature, so the client deadline has to exceed several of
    # those or it would manufacture a transport error while plrd is still
    # legitimately working.  Derived from the daemon's number, in
    # tests/test_recovery_deadlines.py, which parses it out of the Rust
    # source; here we only pin the ordering against the other constants.
    assert daemon_link.EXECUTE_TIMEOUT > daemon_link.DRYRUN_TIMEOUT
    assert daemon_link.EXECUTE_TIMEOUT > 2 * 15 * 60.0


def test_call_rejects_invalid_timeout():
    link = canned_link(ok_bytes())
    with pytest.raises(ValueError, match="out of range"):
        link.call("ping", timeout=0.001)


# --- request/response framing -----------------------------------------


def test_request_is_single_line_json_with_cmd_and_args():
    log = []
    link = canned_link(ok_bytes(), request_log=log)
    link.call("recover_execute", {"confirm": True, "step": False}, timeout=1.0)
    (raw,) = log
    assert raw.endswith(b"\n")
    assert raw.count(b"\n") == 1
    assert json.loads(raw.decode("utf-8")) == {
        "cmd": "recover_execute",
        "args": {"confirm": True, "step": False},
    }


def test_args_default_to_empty_object():
    log = []
    link = canned_link(ok_bytes(), request_log=log)
    link.call("ping", timeout=1.0)
    assert json.loads(log[0].decode("utf-8")) == {"cmd": "ping", "args": {}}


def test_good_response_parsed():
    link = canned_link(ok_bytes("report here", {"stops": 2}))
    resp = link.call("status", timeout=1.0)
    assert resp == {"ok": True, "text": "report here", "data": {"stops": 2}}


def test_response_missing_optional_fields_defaults():
    link = canned_link(b'{"ok": false}\n')
    resp = link.call("status", timeout=1.0)
    assert resp == {"ok": False, "text": "", "data": {}}


def test_malformed_json_is_daemon_error():
    link = canned_link(b"not json at all\n")
    with pytest.raises(daemon_link.DaemonError, match="malformed"):
        link.call("status", timeout=1.0)


def test_missing_ok_field_is_daemon_error():
    link = canned_link(b'{"text": "hi"}\n')
    with pytest.raises(daemon_link.DaemonError, match="boolean 'ok'"):
        link.call("status", timeout=1.0)


def test_non_bool_ok_is_daemon_error():
    link = canned_link(b'{"ok": 1, "text": "hi"}\n')
    with pytest.raises(daemon_link.DaemonError, match="boolean 'ok'"):
        link.call("status", timeout=1.0)


def test_non_string_text_is_daemon_error():
    link = canned_link(b'{"ok": true, "text": 5}\n')
    with pytest.raises(daemon_link.DaemonError, match="non-string 'text'"):
        link.call("status", timeout=1.0)


def test_non_object_data_is_daemon_error():
    link = canned_link(b'{"ok": true, "data": [1]}\n')
    with pytest.raises(daemon_link.DaemonError, match="non-object 'data'"):
        link.call("status", timeout=1.0)


def test_connection_closed_before_newline_is_daemon_error():
    link = canned_link(b"", close_early=True)
    with pytest.raises(daemon_link.DaemonError, match="closed the connection"):
        link.call("status", timeout=1.0)


def test_slow_server_times_out():
    link = canned_link(ok_bytes(), delay=1.0)
    with pytest.raises(daemon_link.DaemonError, match="did not answer"):
        link.call("status", timeout=0.1)


def test_oversized_response_rejected(monkeypatch):
    monkeypatch.setattr(daemon_link, "MAX_RESPONSE_BYTES", 64)
    link = canned_link(b"x" * 200 + b"\n")
    with pytest.raises(daemon_link.DaemonError, match="exceeded 64 bytes"):
        link.call("status", timeout=1.0)


def test_socket_error_during_io_is_daemon_error():
    def connect(path, timeout):
        client, server = socket.socketpair()
        # Both ends fully closed: sendall raises OSError (not a
        # timeout), which must surface as a console-ready DaemonError.
        server.close()
        client.close()
        return client

    link = daemon_link.DaemonLink("/run/fake/plrd.sock", connect_factory=connect)
    with pytest.raises(daemon_link.DaemonError, match="socket error"):
        link.call("status", timeout=1.0)


def test_ping_true_on_ok_false_on_error():
    assert canned_link(ok_bytes()).ping() is True
    assert canned_link(b"", close_early=True).ping() is False


# --- real AF_UNIX transport (default connect factory) ------------------
# CPython on Windows has no AF_UNIX; these run on POSIX and in the WSL
# parity run (see README dev notes).


@pytest.mark.skipif(not HAS_AF_UNIX, reason="python build lacks AF_UNIX")
def test_af_unix_end_to_end(tmp_path):
    path = str(tmp_path / "plrd.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(path)
    server.listen(1)

    def serve():
        conn, _ = server.accept()
        buf = b""
        while b"\n" not in buf:
            buf += conn.recv(4096)
        assert json.loads(buf.decode("utf-8"))["cmd"] == "ping"
        conn.sendall(ok_bytes("pong"))
        conn.close()

    thread = threading.Thread(target=serve)
    thread.daemon = True
    thread.start()
    try:
        link = daemon_link.DaemonLink(path)
        resp = link.call("ping", timeout=daemon_link.PING_TIMEOUT)
        assert resp["ok"] is True and resp["text"] == "pong"
    finally:
        server.close()
        thread.join(timeout=2.0)


@pytest.mark.skipif(not HAS_AF_UNIX, reason="python build lacks AF_UNIX")
def test_af_unix_connect_refused_names_service(tmp_path):
    path = str(tmp_path / "nope.sock")
    link = daemon_link.DaemonLink(path)
    with pytest.raises(daemon_link.DaemonError) as excinfo:
        link.call("ping", timeout=1.0)
    message = str(excinfo.value)
    assert "not reachable" in message
    assert path in message
    assert "systemctl status plrd" in message


@pytest.mark.skipif(HAS_AF_UNIX, reason="covers the no-AF_UNIX guard")
def test_missing_af_unix_support_is_daemon_error():
    with pytest.raises(daemon_link.DaemonError, match="AF_UNIX"):
        daemon_link._default_connect("/run/plrd.sock", 1.0)


# --- PLR_STATUS --------------------------------------------------------


class StubDaemon:
    def __init__(self, response=None, error=None):
        self.response = response
        self.error = error
        self.calls = []

    def call(self, cmd, args=None, timeout=None):
        self.calls.append((cmd, args, timeout))
        if self.error is not None:
            raise self.error
        return self.response


def test_plr_status_reports_plugin_and_daemon(plugin, run_cmd, pump):
    plugin.daemon = StubDaemon(
        response={"ok": True, "text": "armed\nwal: 3 segments", "data": {}}
    )
    plugin.note_pending_save("drag_speed")
    gcode = run_cmd("PLR_STATUS")
    # The plugin's own state is printed by the HANDLER, synchronously...
    out = gcode.responses[-1]
    assert "probe_method: tap" in out
    assert "self_locking_z attested: no" in out
    assert "not measured" in out
    assert "probe_speed: 1.5" in out
    assert "drag_speed: 20 [awaiting SAVE_CONFIG]" in out
    assert "recovery: idle" in out
    # ...and the daemon's block arrives afterwards, from the worker.
    assert "armed" not in out
    assert pump() == 1
    daemon_block = gcode.responses[-1]
    assert "armed" in daemon_block and "wal: 3 segments" in daemon_block
    assert plugin.daemon.calls == [("status", None, daemon_link.STATUS_TIMEOUT)]


def test_plr_status_with_daemon_down_still_reports_plugin(plugin, run_cmd, pump):
    plugin.daemon = StubDaemon(
        error=daemon_link.DaemonError(
            "plrd not reachable at /x — is the service running? (systemctl status plrd)"
        )
    )
    gcode = run_cmd("PLR_STATUS")
    assert "probe_method: tap" in gcode.responses[-1]
    assert pump() == 1
    # A failure a worker discovers becomes klippy's own '!!' error line
    # (there is no gcmd left to raise on).
    assert "not reachable" in gcode.raw_responses[-1]
    assert gcode.raw_responses[-1].startswith("!! ")


def test_plr_status_refuses_a_second_query_while_one_is_in_flight(plugin, run_cmd):
    # One channel, one call: an operator holding the button down must not
    # fork the conversation or spawn threads against a hung daemon.
    release = threading.Event()
    started = []

    class Hanging:
        def call(self, cmd, args=None, timeout=None):
            started.append(cmd)
            release.wait(5.0)
            return {"ok": True, "text": "late", "data": {}}

    plugin.daemon = Hanging()
    try:
        gcode = run_cmd("PLR_STATUS")
        for _ in range(400):
            if started:
                break
            time.sleep(0.005)
        assert started == ["status"]
        run_cmd("PLR_STATUS")
        assert "already in flight" in gcode.responses[-1]
        assert started == ["status"]
    finally:
        release.set()


def test_plr_status_shows_probe_resolution_and_accel_chip(
    fake_printer, plr_config, run_cmd
):
    import plr as plr_pkg

    plugin = plr_pkg.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345",
                "probe_resolution": "0.015",
            }
        )
    )
    plugin.daemon = StubDaemon(response={"ok": True, "text": "", "data": {}})
    out = run_cmd("PLR_STATUS").responses[-1]
    assert "accel_chip: adxl345" in out
    assert "probe_resolution: 0.015000 mm" in out


# --- PLR_RECOVER -------------------------------------------------------


def test_recover_defaults_to_dryrun(plugin, run_cmd, pump):
    plugin.daemon = StubDaemon(
        response={"ok": True, "text": "would resume at layer 12", "data": {}}
    )
    gcode = run_cmd("PLR_RECOVER")
    # The handler returns before the daemon has answered: nothing about the
    # plan can be in the console yet.
    assert "would resume" not in "\n".join(gcode.responses)
    assert pump() == 1
    assert plugin.daemon.calls == [("recover_dryrun", None, daemon_link.DRYRUN_TIMEOUT)]
    assert "would resume at layer 12" in gcode.responses[-1]


def test_recover_dryrun_failure_prints_then_errors(plugin, run_cmd, pump):
    plugin.daemon = StubDaemon(
        response={"ok": False, "text": "machine validation failed", "data": {}}
    )
    gcode = run_cmd("PLR_RECOVER")
    assert pump() == 1
    assert "machine validation failed" in gcode.responses[-1]
    assert "dry run reported failure" in gcode.raw_responses[-1]


def test_recover_execute_without_confirm_refuses_client_side(plugin, run_cmd):
    plugin.daemon = StubDaemon(response={"ok": True, "text": "", "data": {}})
    with pytest.raises(fake_klippy.FakeCommandError, match="CONFIRM=YES"):
        run_cmd("PLR_RECOVER", EXECUTE=1)
    # Refusal happens before any daemon traffic.
    assert plugin.daemon.calls == []


@pytest.mark.parametrize("confirm", ["yes", "Y", "TRUE", "NO"])
def test_recover_execute_wrong_confirm_refuses(plugin, run_cmd, confirm):
    plugin.daemon = StubDaemon(response={"ok": True, "text": "", "data": {}})
    with pytest.raises(fake_klippy.FakeCommandError, match="CONFIRM=YES"):
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM=confirm)
    assert plugin.daemon.calls == []


def test_recover_execute_asks_to_be_consulted_at_confirm_points(plugin, run_cmd, pump):
    # THE POINT OF THE BRANCH: `on_confirm: "ask"` is what makes a
    # Confirmable diagnosis pause instead of aborting
    # (crates/plrd/src/ctrlsock.rs:612-627).  `step` is absent because the
    # daemon refuses it outright (:603-605).
    plugin.daemon = StubDaemon(
        response={
            "ok": True,
            "text": "recovery complete",
            "data": {"outcome": "completed", "exit": 0},
        }
    )
    gcode = run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert plugin.daemon.calls == [
        (
            "recover_execute",
            {"confirm": True, "on_confirm": "ask"},
            daemon_link.EXECUTE_TIMEOUT,
        )
    ]
    assert "WILL MOVE" in gcode.responses[-1]
    assert pump() == 1
    assert "recovery complete" in "\n".join(gcode.responses)


def test_recover_execute_step_mode_refuses_and_names_the_config_key(plugin, run_cmd):
    # STEP=1 could only ever produce plrd's `per-step mode is CLI-only`
    # refusal (ctrlsock.rs:603-605).  Refuse locally and name the key that
    # actually delivers per-step confirmation over the socket.
    plugin.daemon = StubDaemon(response={"ok": True, "text": "", "data": {}})
    with pytest.raises(
        fake_klippy.FakeCommandError, match="debug_confirm_each_step"
    ) as excinfo:
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES", STEP=1)
    assert "not set at the moment" in str(excinfo.value)
    assert plugin.daemon.calls == []


def test_recover_execute_step_mode_says_when_the_key_is_already_set(
    fake_printer, plr_config, run_cmd
):
    import plr as plr_pkg

    plugin = plr_pkg.load_config(
        plr_config(options={"debug_confirm_each_step": "True"})
    )
    plugin.daemon = StubDaemon(response={"ok": True, "text": "", "data": {}})
    with pytest.raises(fake_klippy.FakeCommandError, match="already set"):
        run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES", STEP=1)
    assert plugin.daemon.calls == []


def test_recover_execute_failure_is_reported_from_the_worker(plugin, run_cmd, pump):
    plugin.daemon = StubDaemon(
        response={
            "ok": False,
            "text": "transcript mismatch",
            "data": {"outcome": "aborted-or-refused", "exit": 1},
        }
    )
    gcode = run_cmd("PLR_RECOVER", EXECUTE=1, CONFIRM="YES")
    assert pump() == 1
    joined = "\n".join(gcode.responses)
    assert "transcript mismatch" in joined
    assert "did not complete" in joined
    assert plugin.recovery.state() == "idle"


def test_recover_dryrun_daemon_down_is_reported_from_the_worker(plugin, run_cmd, pump):
    plugin.daemon = StubDaemon(
        error=daemon_link.DaemonError("plrd not reachable at /x")
    )
    gcode = run_cmd("PLR_RECOVER")
    assert pump() == 1
    assert "not reachable" in gcode.raw_responses[-1]


# --- environment note --------------------------------------------------


def test_report_where_socket_tests_ran():
    # Documents (in the test log) which transport the suite exercised;
    # the WSL parity run covers the AF_UNIX leg on Windows dev machines.
    assert isinstance(HAS_AF_UNIX, bool)
    if not HAS_AF_UNIX and os.name != "nt":
        pytest.fail("AF_UNIX missing on a POSIX host — unexpected")
