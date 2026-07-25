"""The reactor must stay free while plrd drives the machine.

This is the file that proves the branch.  It runs the shipped plugin
against a REAL reactor loop (tests/reactor_harness.py — one thread, inline
dispatch, the g-code mutex) and a REAL plrd control socket
(tests/fake_plrd.py — loopback TCP, the verbatim protocol), and measures
how long plugin work holds that thread.

THE NUMBERS IT IS MEASURED AGAINST, from klippy's own sources:

* ``MAX_HEAT_TIME = 3.0`` (klippy/extras/heaters.py:14) is armed on the
  MCU for every heater pin (``setup_max_duration``, heaters.py:62).  An
  MCU pwm pin left at a non-default value with no further update inside
  that window SHUTS THE MCU DOWN (src/pwmcmds.c:45-53 arms
  ``pwm_end_event``).
* ``MAX_MAINTHREAD_TIME = 5.0`` (heaters.py:17) is the deadline
  ``Heater.set_pwm`` compares against before it forces the PWM value to
  zero (heaters.py:72-74), and it is refreshed ONLY from ``Heater.stats``
  on the reactor (heaters.py:138-141).  A reactor stalled past it drops
  every heater to 0 % with its target still set, which
  ``verify_heater`` can escalate to a shutdown
  (klippy/extras/verify_heater.py:86-91).

A recovery is exactly the window where this bites: the plan sets the bed
temperature first and holds for the probe temperature before any motion,
so heaters are active for the whole of ``recover_execute`` by design.

WHAT IS NOT SIMULATED: no heater, no MCU, no watchdog.  The harness
measures reactor unavailability — the input those watchdogs consume — and
the assertions compare it against klippy's thresholds.  The control test
(``test_a_blocking_call_in_a_handler_stalls_the_reactor_past_the_mcu_
watchdog``) shows the measurement bites by producing a real stall with the
real blocking client, so nothing here is a tautology over a constant.
"""

import threading

import fake_klippy
import fake_plrd
import pytest
from reactor_harness import MiniGCode, MiniReactor, MoonrakerBridge

import plr
from plr import daemon_link

# klippy/extras/heaters.py:14 and :17.  Cited, not simulated: see the module
# docstring.  Used as the thresholds MEASURED stalls are compared against.
KLIPPER_MAX_HEAT_TIME = 3.0
KLIPPER_MAX_MAINTHREAD_TIME = 5.0

# What "the reactor stayed free" means here: no single piece of plugin work
# may hold the thread for even a tenth of the MCU watchdog.  Handler work in
# this flow is a few dictionary lookups and some console output, so the real
# figure is sub-millisecond; the margin is for a loaded CI box.
STALL_BUDGET = KLIPPER_MAX_HEAT_TIME / 10.0

# How long the fake daemon takes to answer.  Longer than the stall budget by
# an order of magnitude, so a handler that waited for it could not possibly
# pass, and short enough to keep the suite quick.
DAEMON_LATENCY = 1.0


class Harness:
    """A plugin wired into the MiniReactor with a real plrd socket."""

    def __init__(self, tmp_path, plrd, options=None):
        self.reactor = MiniReactor()
        self.gcode = MiniGCode()
        self.printer = fake_klippy.FakePrinter()
        self.printer.reactor = self.reactor
        self.printer.add_object("gcode", self.gcode)
        self.printer.add_object("configfile", fake_klippy.FakeConfigfile())
        self.printer.add_object("toolhead", fake_klippy.FakeToolhead())
        self.printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
        merged = {"wal_dir": str(tmp_path / "wal")}
        merged.update(options or {})
        config = fake_klippy.FakeConfig(
            self.printer,
            name="plr",
            options=merged,
            sections={
                "force_move": {"enable_force_move": "true"},
                "probe": {"pin": "^PA1", "z_offset": "0.5"},
                "stepper_z": {
                    "step_pin": "PF11",
                    "dir_pin": "!PH1",
                    "enable_pin": "!PA0",
                    "position_min": "-2",
                },
                "printer": {"kinematics": "cartesian"},
            },
        )
        self.plugin = plr.load_config(config)
        # The shipped client, over a real socket, with the loopback
        # connect_factory injected (production uses AF_UNIX).
        self.plugin.daemon = daemon_link.DaemonLink(
            "/run/fake/plrd.sock", connect_factory=plrd.connect_factory()
        )
        self.bridge = MoonrakerBridge(self.reactor, self.gcode)

    def operator(self, line):
        """An operator command arriving the way klippy delivers one.

        klippy/webhooks.py:238-262 queues the request as a reactor callback
        and :447-448 runs it with ``GCodeDispatch.run_script``, which holds
        the g-code mutex for the whole script (klippy/gcode.py:239-241).
        The harness runs it inside the reactor's own timing, so the
        handler's cost lands in ``reactor.max_stall``.
        """
        done = []

        def run(eventtime):
            try:
                self.gcode.run_script(line, source="operator")
            finally:
                done.append(True)

        self.reactor.register_async_callback(run)
        assert self.reactor.run_until(lambda: done, timeout=10.0), (
            "the operator command never ran"
        )

    def push_from_daemon(self, script, timeout=2.0):
        """Have plrd push g-code at klippy, turning the loop while it waits.

        The push has to come from another thread (it does in production:
        plrd is another process), and the reactor has to keep running or
        nothing could service it — which is the whole point.
        """
        result = []
        thread = threading.Thread(
            target=lambda: result.append(self.bridge.send_script(script, timeout))
        )
        thread.daemon = True
        thread.start()
        self.reactor.run_until(lambda: result, timeout=timeout + 1.0)
        thread.join(timeout=1.0)
        return result[0] if result else None

    def close(self):
        self.reactor.close()


@pytest.fixture
def harness(tmp_path):
    made = []

    def build(plrd, options=None):
        h = Harness(tmp_path, plrd, options=options)
        made.append(h)
        return h

    yield build
    for h in made:
        h.close()


def _executing_daemon(bridge_holder, delay=DAEMON_LATENCY, script=None):
    """A plrd that pushes g-code at klippy while answering, like a recovery.

    ``on_request`` runs on the daemon's thread DURING ``recover_execute``,
    which is where a real recovery's ``printer.gcode.script`` calls happen
    (crates/plrd/src/executor.rs:1013 ``send_step_commands`` /
    crates/plrd/src/moonraker.rs:167-172).
    """

    def on_request(cmd, args):
        if cmd == "recover_execute":
            bridge_holder[0].send_script("M140 S60", timeout=DAEMON_LATENCY * 3)

    return fake_plrd.FakePlrd(script=script, delay=delay, on_request=on_request)


# --- the fix ----------------------------------------------------------


def test_the_execute_handler_leaves_the_reactor_free_for_plrds_own_gcode(harness):
    holder = [None]
    plrd = _executing_daemon(
        holder,
        script={
            "recover_execute": [
                {
                    "ok": True,
                    "text": "recover: plan complete; print resumed",
                    "data": {"outcome": "completed", "exit": 0},
                }
            ]
        },
    )
    h = harness(plrd)
    holder[0] = h.bridge
    try:
        h.operator("PLR_RECOVER EXECUTE=1 CONFIRM=YES")
        # The handler is already back: the recovery is still in flight, and
        # nothing about its outcome exists yet.
        assert h.plugin.recovery.state() == "running"
        assert h.gcode.mutex_held_by is None
        # plrd, meanwhile, got klippy to run its step commands — WHILE the
        # recovery was in flight.  On the pre-fix code this could not
        # happen: the handler held both the reactor and the mutex.
        assert h.reactor.run_until(
            lambda: h.bridge.completed, timeout=DAEMON_LATENCY * 3
        ), "plrd's g-code never ran while the recovery was in flight"
        assert ("plrd", "M140 S60") in h.gcode.scripts_run
        assert h.bridge.refused == []
        # ...and then the report arrives from a reactor callback.
        assert h.reactor.run_until(
            lambda: h.plugin.recovery.state() == "idle", timeout=DAEMON_LATENCY * 4
        )
        assert any("plan complete" in line for line in h.gcode.responses)
        # THE MEASUREMENT: the daemon took DAEMON_LATENCY to answer, and no
        # single piece of work held the reactor for even a fraction of the
        # MCU heater watchdog.
        assert h.reactor.max_stall < STALL_BUDGET, (
            "reactor held for %.3fs (budget %.3fs, klippy's MCU heater "
            "watchdog is %.1fs and its main-thread deadline %.1fs)"
            % (
                h.reactor.max_stall,
                STALL_BUDGET,
                KLIPPER_MAX_HEAT_TIME,
                KLIPPER_MAX_MAINTHREAD_TIME,
            )
        )
    finally:
        plrd.close()


def test_the_whole_confirm_point_conversation_never_holds_the_reactor(harness):
    pause = {
        "ok": False,
        "text": "recover: PAUSED at step 9 [z-confirm-standoff] awaiting confirmation",
        "data": {
            "outcome": "awaiting_confirmation",
            "resume_token": "plrc-abc-1",
            "confirm_kind": "z-height",
            "step": 9,
            "phase": "z-confirm-standoff",
            "diagnosis": {
                "code": "z_confirm_before_resume",
                "tier": "confirmable",
                "what": "the toolhead is standing off at Z 0.6 mm",
                "why": "confirm_z_before_resume is set",
                "suggested_fix": "answer continue if the standoff looks right",
                "measured": {
                    "quantity": "toolhead.position.2",
                    "value": 0.6,
                    "unit": "mm",
                },
                "expected": None,
                "override_key": None,
            },
            "detail": {
                "standoff_target_z": 0.6,
                "live_toolhead_z": 0.6,
                "derivation": "true_Z = z_prev_top 0.4 + (halt_Z - trigger_Z)",
            },
        },
    }
    holder = [None]
    plrd = _executing_daemon(
        holder,
        script={
            "recover_execute": [pause],
            "recover_confirm": [
                {
                    "ok": True,
                    "text": "recover: plan complete; print resumed",
                    "data": {"outcome": "completed", "exit": 0},
                }
            ],
        },
    )
    h = harness(plrd)
    holder[0] = h.bridge
    try:
        h.operator("PLR_RECOVER EXECUTE=1 CONFIRM=YES")
        assert h.reactor.run_until(
            lambda: h.plugin.recovery.is_awaiting(), timeout=DAEMON_LATENCY * 4
        ), "the confirm-point never reached the reactor"
        # The prompt really was rendered, on a real reactor, from a worker
        # callback.
        assert "action:prompt_show" in h.gcode.responses
        assert any(
            "PLR_RECOVER_CONTINUE" in line and not line.startswith("action:")
            for line in h.gcode.responses
        )
        # The operator answers with the console command the prompt named.
        h.operator("PLR_RECOVER_CONTINUE")
        assert h.reactor.run_until(
            lambda: h.plugin.recovery.state() == "idle", timeout=DAEMON_LATENCY * 4
        )
        assert any("plan complete" in line for line in h.gcode.responses)
        assert plrd.requests[0][0] == "recover_execute"
        assert plrd.requests[0][1] == {"confirm": True, "on_confirm": "ask"}
        assert plrd.requests[1] == (
            "recover_confirm",
            {"token": "plrc-abc-1", "answer": "continue"},
        )
        assert h.reactor.max_stall < STALL_BUDGET, h.reactor.max_stall
    finally:
        plrd.close()


def test_no_reactor_callback_of_this_flow_takes_the_gcode_mutex(harness):
    # Progress, prompts and reports go out through respond_info, which takes
    # NO mutex (klippy/gcode.py:247-254).  If any of them ran g-code they
    # would queue behind plrd's own motion — and could deadlock against it.
    holder = [None]
    plrd = _executing_daemon(
        holder,
        script={
            "recover_execute": [
                {
                    "ok": False,
                    "text": "recover: ABORTED at step 4",
                    "data": {"outcome": "aborted-or-refused", "exit": 1},
                }
            ]
        },
    )
    h = harness(plrd)
    holder[0] = h.bridge
    try:
        h.operator("PLR_RECOVER EXECUTE=1 CONFIRM=YES")
        assert h.reactor.run_until(
            lambda: h.plugin.recovery.state() == "idle", timeout=DAEMON_LATENCY * 4
        )
        # Only the operator's own command and plrd's step commands ever took
        # the mutex.
        sources = {source for source, _line in h.gcode.scripts_run}
        assert sources <= {"operator", "plrd"}, sources
        assert h.gcode.deferred_scripts == []
        assert h.gcode.mutex_held_by is None
    finally:
        plrd.close()


def test_a_hung_daemon_never_stalls_the_reactor(harness):
    # Alive, connected, never answering — the case a timeout is for.  The
    # worker sits in recv; klippy carries on.
    plrd = fake_plrd.FakePlrd(hang=True)
    h = harness(plrd)
    try:
        h.operator("PLR_STATUS")
        h.operator("PLR_RECOVER EXECUTE=1 CONFIRM=YES")
        # Keep the loop turning for well past the MCU watchdog...
        h.reactor.run_until(lambda: False, timeout=KLIPPER_MAX_HEAT_TIME + 0.5)
        # ...and klippy was available throughout.
        assert h.reactor.max_stall < STALL_BUDGET, h.reactor.max_stall
        assert h.push_from_daemon("M105") is True
        assert ("plrd", "M105") in h.gcode.scripts_run
        # The plugin still knows the recovery is in flight, and refuses a
        # second one rather than forking the conversation.
        assert h.plugin.recovery.state() == "running"
        h.operator("PLR_RECOVER EXECUTE=1 CONFIRM=YES")
        assert any("already in flight" in line for line in h.gcode.raw_responses)
    finally:
        plrd.close()


# --- the control: the defect this branch fixes, measured --------------


def test_a_blocking_call_in_a_handler_stalls_the_reactor_past_the_mcu_watchdog(
    harness,
):
    """The pre-fix shape, measured on a real loop.

    This is the same client (``DaemonLink.call``) that the plugin used to
    call directly from ``cmd_PLR_RECOVER`` / the wizard's execute, against a
    daemon that does not answer.  It exists so the harness's measurement is
    known to BITE: if this stall did not exceed klippy's watchdog, the
    passing tests above would prove nothing.

    It also shows the second half of the defect: plrd's own g-code cannot
    run while the handler holds the reactor and the mutex, and then runs
    AFTER the handler has failed — the machine moving after the operator
    was told the recovery failed.
    """
    # Deliberately just over klippy's MCU heater watchdog, to keep the test
    # short while crossing the threshold that matters.  The shipped code
    # used RECOVER_TIMEOUT = 120 s here, i.e. 40x this.
    blocking_timeout = KLIPPER_MAX_HEAT_TIME + 0.2
    holder = [None]
    pushed = []

    def on_request(cmd, args):
        # Runs on the daemon's thread, once the blocking handler's request
        # has arrived — i.e. provably while the handler holds the reactor.
        pushed.append(holder[0].send_script("M140 S60", timeout=blocking_timeout))

    plrd = fake_plrd.FakePlrd(hang=True, on_request=on_request)
    h = harness(plrd)
    holder[0] = h.bridge

    def blocking_handler(gcmd):
        # Exactly what the old handler did: a blocking daemon call inside a
        # g-code handler.
        try:
            h.plugin.daemon.call(
                "recover_execute",
                {"confirm": True},
                timeout=blocking_timeout,
            )
        except daemon_link.DaemonError as e:
            gcmd.respond_info("PLR_RECOVER: %s" % (e,))

    h.gcode.register_command("PLR_RECOVER_BLOCKING", blocking_handler)
    try:
        h.operator("PLR_RECOVER_BLOCKING")

        # THE DEFECT, MEASURED: one handler held klippy's only thread for
        # longer than the MCU's heater watchdog.
        assert h.reactor.max_stall > KLIPPER_MAX_HEAT_TIME, (
            "expected a stall past klippy's %.1fs MCU heater watchdog, "
            "measured %.3fs" % (KLIPPER_MAX_HEAT_TIME, h.reactor.max_stall)
        )
        # plrd's script did not run while the handler was blocked...
        assert pushed == [False]
        assert ("plrd", "M140 S60") not in h.gcode.scripts_run
        # ...and the operator has already been told it failed.
        assert any("PLR_RECOVER:" in line for line in h.gcode.responses)
        # THEN the queued script runs, once the reactor is free again: the
        # machine moves after the failure was reported.
        h.reactor.run_until(lambda: h.bridge.completed, timeout=2.0)
        assert ("plrd", "M140 S60") in h.gcode.scripts_run
    finally:
        plrd.close()


def test_the_shipped_handlers_are_the_non_blocking_ones(harness):
    # The guard that keeps the fix from being undone by a "small
    # simplification": drive every registered PLR command that talks to plrd
    # against a daemon that never answers, and require each handler to
    # return in a fraction of the MCU watchdog.
    plrd = fake_plrd.FakePlrd(hang=True)
    try:
        for line in (
            "PLR_STATUS",
            "PLR_RECOVER",
            "PLR_RECOVER EXECUTE=1 CONFIRM=YES",
            "PLR_WIZARD_START",
        ):
            # A fresh plugin per command: each has its own single-flight
            # channel, and this test is about the HANDLER's cost, not about
            # which channel is busy.
            h = harness(plrd)
            h.operator(line)
            assert h.reactor.max_stall < STALL_BUDGET, (
                "%s held the reactor for %.3fs (budget %.3fs, klippy's MCU "
                "heater watchdog is %.1fs)"
                % (line, h.reactor.max_stall, STALL_BUDGET, KLIPPER_MAX_HEAT_TIME)
            )
            # Nothing raised into the loop, either.
            assert h.gcode.internal_errors == []
    finally:
        plrd.close()
