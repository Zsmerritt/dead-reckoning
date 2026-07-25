"""The confirm-deadline interlock: the plugin must outlast the daemon.

THE STANDING REVIEW QUESTION this file answers: *for every limit one
component enforces, who sets the value it is compared against, and do they
meet at exactly the same number?*

plrd bounds an unanswered confirm-point itself and aborts cleanly on
expiry (crates/plrd/src/executor.rs ``ask``).  The plugin also has to stop
believing a pause is live at some point, or an abandoned dialog wedges the
flow forever.  If the plugin's deadline were SHORTER, the operator would
be told the question expired while plrd was still holding the machine and
would still accept an answer; if they were EQUAL, a race would decide.  So
the plugin's deadline must be strictly longer, by named headroom, and it
must be DERIVED from plrd's value rather than restating it.

WHY THIS TEST PARSES RUST.  The numbers being compared live in the Rust
crates; a python copy of them asserted against itself would prove nothing
and would go stale in silence.  Extraction is checked for vacuity, the
same discipline tests/test_daemon_keys.py uses for the ``[plr]`` key set.
"""

import os
import re

import pytest

from plr import daemon_link, recovery

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_TESTS_DIR))
_BUILD_RS = os.path.join(_REPO_ROOT, "crates", "plr-recovery", "src", "build.rs")
_EXECUTOR_RS = os.path.join(_REPO_ROOT, "crates", "plrd", "src", "executor.rs")


def _read(path):
    assert os.path.isfile(path), (
        "cannot find %s — this interlock is derived from the daemon's own "
        "constants; if the layout moved, update the path, never delete the "
        "check" % (path,)
    )
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def _rust_f64(source, name, path):
    """The value of ``pub const <name>: f64 = <number>;`` in ``source``."""
    match = re.search(
        r"pub const %s:\s*f64\s*=\s*([0-9_]+(?:\.[0-9]+)?)" % (re.escape(name),),
        source,
    )
    assert match is not None, (
        "%s no longer defines `pub const %s: f64` — the plugin's confirm "
        "deadline is derived from it, so this must be reconciled by hand, "
        "not skipped" % (path, name)
    )
    return float(match.group(1).replace("_", ""))


def daemon_confirm_constants():
    """plrd's own confirm-timeout numbers, read out of the Rust source."""
    build = _read(_BUILD_RS)
    executor = _read(_EXECUTOR_RS)
    # `DEFAULT_CONFIRM_TIMEOUT: Duration = Duration::from_mins(10)`
    minutes = re.search(
        r"pub const DEFAULT_CONFIRM_TIMEOUT:\s*Duration\s*=\s*"
        r"Duration::from_mins\((\d+)\)",
        executor,
    )
    assert minutes is not None, (
        "crates/plrd/src/executor.rs no longer declares DEFAULT_CONFIRM_TIMEOUT "
        "as Duration::from_mins(N) — reconcile the plugin's ceiling by hand"
    )
    return {
        "default_s": _rust_f64(build, "CONFIRM_TIMEOUT_DEFAULT_S", _BUILD_RS),
        "min_s": _rust_f64(build, "CONFIRM_TIMEOUT_MIN_S", _BUILD_RS),
        "max_s": _rust_f64(build, "CONFIRM_TIMEOUT_MAX_S", _BUILD_RS),
        "executor_default_s": float(minutes.group(1)) * 60.0,
    }


# --- the extraction itself is not allowed to rot ----------------------


def test_the_extraction_is_not_vacuous():
    values = daemon_confirm_constants()
    # Every number must be a plausible operator-scale deadline, so a regex
    # that started matching the wrong literal fails here rather than
    # silently weakening the interlock.
    assert 1.0 <= values["min_s"] < values["default_s"] < values["max_s"] <= 86400.0
    # The two spellings of plrd's default (the plan-config constant and the
    # executor's Duration) must agree with each other; if they ever diverge
    # the daemon has two defaults and this plugin cannot reason about either.
    assert values["default_s"] == values["executor_default_s"]


# --- the interlock ----------------------------------------------------


def test_the_ceiling_is_the_daemons_own_band_maximum():
    values = daemon_confirm_constants()
    assert recovery.DAEMON_CONFIRM_CEILING_S == values["max_s"]
    # ...and it really is an upper bound on the deadline plrd may be using
    # when it has not told us: its unreported default sits below it.
    assert values["default_s"] <= recovery.DAEMON_CONFIRM_CEILING_S


def test_the_headroom_is_positive_so_the_two_deadlines_cannot_coincide():
    assert recovery.CONFIRM_HEADROOM_S > 0.0


@pytest.mark.parametrize(
    "configured",
    [
        pytest.param(None, id="unset"),
        pytest.param(30.0, id="band-minimum"),
        pytest.param(600.0, id="daemon-default-written-out"),
        pytest.param(3600.0, id="band-maximum"),
    ],
)
def test_the_plugin_always_waits_strictly_longer_than_the_daemon(configured):
    daemon, _exact = recovery.daemon_confirm_deadline(configured)
    plugin = recovery.prompt_deadline(configured)
    assert plugin > daemon
    assert plugin - daemon == recovery.CONFIRM_HEADROOM_S


def test_the_unset_case_outlasts_every_deadline_the_daemon_could_be_using():
    values = daemon_confirm_constants()
    plugin = recovery.prompt_deadline(None)
    for daemon_deadline in (
        values["min_s"],
        values["default_s"],
        values["max_s"],
    ):
        assert plugin > daemon_deadline


def test_a_configured_value_is_used_exactly_not_re_derived():
    # The operator's `[plr] confirm_timeout_s` is the SAME value plrd reads
    # from the same printer.cfg section, so the plugin uses it verbatim
    # rather than keeping a second copy of the default.
    assert recovery.daemon_confirm_deadline(45.0) == (45.0, True)
    assert recovery.prompt_deadline(45.0) == 45.0 + recovery.CONFIRM_HEADROOM_S


@pytest.mark.parametrize(
    "configured",
    [
        pytest.param(None, id="unset"),
        pytest.param("600", id="string"),
        pytest.param(True, id="bool"),
        pytest.param(0.0, id="zero"),
        pytest.param(-5.0, id="negative"),
        pytest.param(float("nan"), id="nan"),
        pytest.param(float("inf"), id="inf"),
    ],
)
def test_an_unusable_setting_falls_back_to_the_ceiling_never_to_something_short(
    configured,
):
    # FAIL-SAFE DIRECTION: an unreadable setting must LENGTHEN the plugin's
    # wait, never shorten it, because a short local deadline is the failure
    # that tells the operator a live question is dead.
    deadline, exact = recovery.daemon_confirm_deadline(configured)
    assert exact is False
    assert deadline == recovery.DAEMON_CONFIRM_CEILING_S


# --- what the operator is told about the deadline ---------------------


def test_the_number_is_quoted_only_when_the_plugin_actually_knows_it():
    known = recovery.deadline_text(120.0)
    assert "120 s" in known
    assert "confirm_timeout_s" in known
    unknown = recovery.deadline_text(None)
    # No number at all: telling the operator the wrong deadline is worse
    # than telling them there is one.
    assert not re.search(r"\d", unknown)
    assert "its default" in unknown
    assert "confirm_timeout_s" in unknown


# --- the plugin's own socket deadlines vs the daemon's ----------------


def test_the_execute_deadline_outlasts_the_daemons_step_deadlines():
    # recover_execute answers only when the recovery pauses or finishes
    # (ctrlsock.rs:790-836), and plrd's per-step temperature deadline alone
    # is ExecOptions::temp_timeout.  Parse it rather than trusting a comment.
    executor = _read(_EXECUTOR_RS)
    match = re.search(r"temp_timeout:\s*Duration::from_mins\((\d+)\)", executor)
    assert match is not None, (
        "crates/plrd/src/executor.rs no longer declares temp_timeout as "
        "Duration::from_mins(N) — the client deadline is sized against it"
    )
    temp_timeout_s = float(match.group(1)) * 60.0
    # A stock plan waits for the bed AND then the probe temperature, so one
    # temp_timeout is not the bound; two is the floor of the bound.
    assert daemon_link.EXECUTE_TIMEOUT > 2 * temp_timeout_s
    # And the plugin's own sanity ceiling must not be the thing that
    # truncates it.
    assert daemon_link.EXECUTE_TIMEOUT <= daemon_link.MAX_TIMEOUT


def test_the_status_deadline_is_not_the_thing_protecting_the_reactor():
    # The 5 s status deadline used to sit inside a g-code handler, where it
    # already exceeded klippy's 3 s heater watchdog.  It is unchanged — and
    # that is only safe because it is now spent on a worker thread; this
    # test exists to make the ordering explicit rather than incidental.
    assert daemon_link.STATUS_TIMEOUT > 3.0
    assert daemon_link.MAX_TIMEOUT > daemon_link.STATUS_TIMEOUT
