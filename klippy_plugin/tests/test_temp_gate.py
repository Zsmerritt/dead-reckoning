"""Contact-operation nozzle-temperature gate.

Every PLR command that can bring the nozzle to the part refuses while the
nozzle is hot, single-sourced in ``setup_checks.nozzle_too_hot_message``.
These tests pin the shared helper directly AND drive the full matrix of
all four commands x {current hot, target hot, both cool} through the
registered console commands, proving the gate fires (or does not) at each.
"""

import os

import fake_klippy
import pytest

import plr
from plr import setup_checks

REFUSAL_PHRASE = "Cool the nozzle below"
# A source-contiguous token unique to the refusal message (REFUSAL_PHRASE
# wraps across two source lines), used to prove the gate is single-sourced.
SOURCE_MARKER = "M104 S0"


# --- shared-helper unit tests ----------------------------------------


def _printer_with_extruder(extruder, on_toolhead=False):
    printer = fake_klippy.FakePrinter()
    if on_toolhead:
        printer.add_object("toolhead", fake_klippy.FakeToolhead(extruder=extruder))
    else:
        printer.add_object("toolhead", fake_klippy.FakeToolhead())
        if extruder is not None:
            printer.add_object("extruder", extruder)
    return printer


def test_no_extruder_means_no_temperature_reading():
    printer = fake_klippy.FakePrinter()
    printer.add_object("toolhead", fake_klippy.FakeToolhead())
    assert setup_checks.nozzle_temperatures(printer) is None
    assert setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH") is None


def test_active_extruder_read_via_toolhead_get_extruder():
    ext = fake_klippy.FakeExtruder(temperature=200.0, target=0.0)
    printer = _printer_with_extruder(ext, on_toolhead=True)
    assert setup_checks.nozzle_temperatures(printer) == (200.0, 0.0)


def test_extruder_read_via_primary_object_fallback():
    ext = fake_klippy.FakeExtruder(temperature=200.0, target=0.0)
    printer = _printer_with_extruder(ext, on_toolhead=False)
    assert setup_checks.nozzle_temperatures(printer) == (200.0, 0.0)


def test_missing_fields_read_as_zero():
    ext = fake_klippy.FakeExtruder(report={"temperature": 190.0})  # no 'target'
    printer = _printer_with_extruder(ext)
    assert setup_checks.nozzle_temperatures(printer) == (190.0, 0.0)


def test_empty_status_is_none():
    ext = fake_klippy.FakeExtruder(report={})
    printer = _printer_with_extruder(ext)
    assert setup_checks.nozzle_temperatures(printer) is None
    assert setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH") is None


def test_current_hot_refused():
    ext = fake_klippy.FakeExtruder(temperature=250.0, target=0.0)
    printer = _printer_with_extruder(ext)
    msg = setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH")
    assert msg is not None and REFUSAL_PHRASE in msg and "150" in msg


def test_target_only_hot_refused():
    # The safety hole: cold tip, but commanded to 250 — already on its way.
    ext = fake_klippy.FakeExtruder(temperature=45.0, target=250.0)
    printer = _printer_with_extruder(ext)
    msg = setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_DRAG_PROBE")
    assert msg is not None and "45°C" in msg and "250°C" in msg


def test_both_cool_allowed():
    ext = fake_klippy.FakeExtruder(temperature=45.0, target=0.0)
    printer = _printer_with_extruder(ext)
    assert setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH") is None


def test_exactly_at_threshold_allowed():
    # Strictly greater refuses; equal passes.
    ext = fake_klippy.FakeExtruder(temperature=150.0, target=150.0)
    printer = _printer_with_extruder(ext)
    assert setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH") is None


def test_one_over_threshold_refused():
    ext = fake_klippy.FakeExtruder(temperature=150.5, target=0.0)
    printer = _printer_with_extruder(ext)
    assert setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH") is not None


def test_threshold_honors_configured_max():
    ext = fake_klippy.FakeExtruder(temperature=130.0, target=0.0)
    printer = _printer_with_extruder(ext)
    # Allowed at the 150 default, refused at a tightened 120 limit.
    assert setup_checks.nozzle_too_hot_message(printer, 150.0, "PLR_TOUCH") is None
    assert setup_checks.nozzle_too_hot_message(printer, 120.0, "PLR_TOUCH") is not None


def test_temperature_threshold_is_single_sourced():
    # The refusal text (and thus the threshold comparison behind it) must
    # live in exactly one module — no duplicated per-command logic.
    pkg_dir = os.path.dirname(plr.__file__)
    hits = [
        fn
        for fn in os.listdir(pkg_dir)
        if fn.endswith(".py")
        and SOURCE_MARKER in open(os.path.join(pkg_dir, fn), encoding="utf-8").read()
    ]
    assert hits == ["setup_checks.py"]


# --- full per-command matrix (through the registered commands) --------

# (temperature, target, should_refuse) — current hot, target hot, both cool.
TEMP_CASES = [
    pytest.param(250.0, 0.0, True, id="current-hot"),
    pytest.param(45.0, 250.0, True, id="target-hot"),
    pytest.param(45.0, 0.0, False, id="both-cool"),
]


def _run_expecting(run_cmd, name, should_refuse, **params):
    """Run a command; return the refusal-message-or-None for the temp gate.

    Refused → the raised error carries the nozzle-hot phrase.  Allowed →
    either no error, or a DIFFERENT gate's error (never the temp phrase).
    """
    try:
        run_cmd(name, **params)
    except fake_klippy.FakeCommandError as e:
        message = str(e)
        if should_refuse:
            assert REFUSAL_PHRASE in message, message
        else:
            assert REFUSAL_PHRASE not in message, message
        return
    # No error at all: only acceptable when the gate should have allowed.
    assert not should_refuse


def _touch_plugin(fake_printer, plr_config, temperature, target):
    ext = fake_klippy.FakeExtruder(temperature=temperature, target=target)
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead(extruder=ext))
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    return plr.load_config(plr_config())


def _drag_plugin(fake_printer, plr_config, temperature, target):
    ext = fake_klippy.FakeExtruder(temperature=temperature, target=target)
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead(extruder=ext))
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    fake_printer.add_object("adxl345", fake_klippy.FakeAccelChip(fake_printer))
    return plr.load_config(
        plr_config(options={"probe_method": "adxl_drag", "accel_chip": "adxl345"})
    )


@pytest.mark.parametrize("temperature,target,refuse", TEMP_CASES)
def test_gate_PLR_TOUCH(fake_printer, plr_config, run_cmd, temperature, target, refuse):
    _touch_plugin(fake_printer, plr_config, temperature, target)
    _run_expecting(run_cmd, "PLR_TOUCH", refuse)


@pytest.mark.parametrize("temperature,target,refuse", TEMP_CASES)
def test_gate_PLR_PROBE_TEST(
    fake_printer, plr_config, run_cmd, temperature, target, refuse
):
    _touch_plugin(fake_printer, plr_config, temperature, target)
    # No START= — the temp gate runs before the motion-consent gate, so a
    # hot nozzle refuses even the plan preview.
    _run_expecting(run_cmd, "PLR_PROBE_TEST", refuse)


@pytest.mark.parametrize("temperature,target,refuse", TEMP_CASES)
def test_gate_PLR_DRAG_PROBE(
    fake_printer, plr_config, run_cmd, temperature, target, refuse
):
    plugin = _drag_plugin(fake_printer, plr_config, temperature, target)
    _run_expecting(run_cmd, "PLR_DRAG_PROBE", refuse)
    if refuse:
        # A gate refusal must also surface in status, not just the console.
        assert plugin.last_drag_error is not None
        assert REFUSAL_PHRASE in plugin.last_drag_error


@pytest.mark.parametrize("temperature,target,refuse", TEMP_CASES)
def test_gate_PLR_DRAG_CALIBRATE(
    fake_printer, plr_config, run_cmd, temperature, target, refuse
):
    _drag_plugin(fake_printer, plr_config, temperature, target)
    _run_expecting(run_cmd, "PLR_DRAG_CALIBRATE", refuse)


def test_gate_absent_extruder_never_blocks_touch(fake_printer, plr_config, run_cmd):
    # No extruder wired at all: the gate must not block (nothing to ooze).
    fake_printer.add_object("toolhead", fake_klippy.FakeToolhead())
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    plr.load_config(plr_config())
    try:
        run_cmd("PLR_TOUCH")
    except fake_klippy.FakeCommandError as e:
        assert REFUSAL_PHRASE not in str(e)
