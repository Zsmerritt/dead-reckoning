"""Tests for the PLR_DRAG_CALIBRATE sensitive-first sweep.

The chip script decides WHAT samples arrive per pass (glue); the
classification of those samples is always the real classifier math.  The
load-bearing safety invariant — the command NEVER descends — is asserted
over the recorded manual_move list, including under scripts that would
tempt a descent.
"""

import fake_klippy
import pytest
import stream_fixtures as sf

import plr
from plr import classifier, drag_calibrate, tunables

# Noise floor consistent with sf.quiet(noise=5.0): peak windowed RMS
# ~9.4, so a pure-quiet stream never false-triggers even at knob 100.
NOISE_FLOOR = "8.66"

# good_sections() ships [stepper_z] position_min = -2, so the clear
# standoff floor is -2 + 5 = 3.0.  Start well above it.
START_Z = 6.0
CLEAR_FLOOR = 3.0


def clean(toolhead):
    return sf.quiet(seed=41)


def hard_contact(toolhead):
    return sf.with_contact(sf.quiet(seed=44), amplitude=500.0)


# A moderate burst that reads as contact at high sensitivity (knob 100)
# but is clean at knob 60 and below: the "false-triggers at 100 but not
# <= 60" stream from the acceptance criteria.
def boundary_stream():
    return sf.with_contact(sf.quiet(seed=50), amplitude=22.0)


@pytest.fixture
def calib_setup(fake_printer, plr_config):
    """Plugin + homed toolhead + scripted chip for calibrate tests."""

    def build(
        chip_script=None,
        chip_default=None,
        start_z=START_Z,
        options=None,
        chip_name="adxl345",
    ):
        toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, start_z, 0.0))
        fake_printer.add_object("toolhead", toolhead)
        fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
        chip = fake_klippy.FakeAccelChip(
            fake_printer, script=chip_script, default=chip_default
        )
        fake_printer.add_object(chip_name, chip)
        merged = {
            "probe_method": "adxl_drag",
            "accel_chip": chip_name,
            "noise_floor_rms": NOISE_FLOOR,
        }
        merged.update(options or {})
        merged = {k: v for k, v in merged.items() if v is not None}
        plugin = plr.load_config(plr_config(options=merged))
        return plugin, toolhead, chip

    return build


def commanded_z_values(moves):
    return [m[0][2] for m in moves if len(m[0]) > 2 and m[0][2] is not None]


def lateral_moves(moves):
    return [m for m in moves if m[0][0] is not None]


# ---------------------------------------------------------------------
# Pure helpers


@pytest.mark.parametrize(
    ("knob", "false_triggers", "expected"),
    [
        (100.0, 3, 15),  # badly: 20% of 100 = 20 -> clamped to 15
        (100.0, 1, 10),  # close: 10% of 100 = 10
        (70.0, 2, 14),  # badly: 20% of 70 = 14
        (70.0, 1, 7),  # close: 10% of 70 = 7
        (8.0, 3, 2),  # badly: 20% of 8 = 1 -> clamped up to 2
        (8.0, 1, 2),  # close: 10% of 8 = 0 -> clamped up to 2
    ],
)
def test_knob_step_adaptive_and_clamped(knob, false_triggers, expected):
    assert drag_calibrate.knob_step(knob, false_triggers) == expected


# ---------------------------------------------------------------------
# Safety: the command never descends


def test_never_descends_lifts_to_clear_z_only(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean, start_z=6.0)
    run_cmd("PLR_DRAG_CALIBRATE", START=1, CLEAR_Z=10.0)
    # The only Z move is the upward lift to CLEAR_Z; every pass is lateral
    # (Z=None).  THE safety assertion: no commanded Z is ever below
    # CLEAR_Z, and the minimum commanded Z equals CLEAR_Z exactly.
    zs = commanded_z_values(toolhead.moves)
    assert zs, "expected the upward lift to CLEAR_Z to be recorded"
    assert min(zs) == pytest.approx(10.0)
    assert all(z == pytest.approx(10.0) for z in zs)
    assert all(m[0][2] is None for m in lateral_moves(toolhead.moves))
    assert toolhead.position[2] == pytest.approx(10.0)


def test_never_descends_even_when_contact_scripted(calib_setup, run_cmd):
    """A script that reports contact at every pass (which in a real probe
    would tempt a descent) still never moves Z below CLEAR_Z — calibrate
    has no descent path at all."""
    plugin, toolhead, chip = calib_setup(chip_default=hard_contact, start_z=8.0)
    run_cmd("PLR_DRAG_CALIBRATE", START=1, CLEAR_Z=8.0)
    zs = commanded_z_values(toolhead.moves)
    # No lift needed (CLEAR_Z == current), and definitely no descent.
    assert all(z >= 8.0 for z in zs)
    assert all(m[0][2] is None for m in lateral_moves(toolhead.moves))


def test_refuses_clear_z_below_standoff(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    with pytest.raises(fake_klippy.FakeCommandError, match="clear standoff"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1, CLEAR_Z=2.0)
    assert toolhead.moves == []
    assert plugin.last_drag_calibrate is None


def test_refuses_when_default_clear_z_below_standoff(calib_setup, run_cmd):
    # Current Z below the floor+5 standoff, no CLEAR_Z given -> default is
    # the current Z, which is refused (move up first).
    plugin, toolhead, chip = calib_setup(chip_default=clean, start_z=2.5)
    with pytest.raises(fake_klippy.FakeCommandError, match="clear standoff"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert toolhead.moves == []


def test_refuses_to_descend_to_clear_z(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean, start_z=8.0)
    with pytest.raises(fake_klippy.FakeCommandError, match="never descends"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1, CLEAR_Z=5.0)
    assert toolhead.moves == []


# ---------------------------------------------------------------------
# Sensitive-first direction


def test_accepts_knob_100_when_never_false_triggers(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    run_cmd("PLR_DRAG_CALIBRATE", START=1)
    result = plugin.last_drag_calibrate
    assert result["accepted_knob"] == pytest.approx(100.0)
    assert result["tested"] == [100.0]
    # 3 screen + 6 verify passes, all clean, one candidate.
    assert len(chip.clients) == 9


def test_sensitive_first_stops_at_first_survivor(calib_setup, run_cmd):
    """A stream that false-triggers at knob 100 but not at <= 60: the
    sweep steps DOWN and accepts the highest surviving knob, never
    testing a lower knob than necessary."""
    stream = boundary_stream()
    floor = float(NOISE_FLOOR)
    # Premise, proved against the real classifier: contact at 100, clean
    # at 60 and below.
    assert classifier.classify_pass(stream, floor, 100.0).contact is True
    assert classifier.classify_pass(stream, floor, 60.0).contact is False

    plugin, toolhead, chip = calib_setup(chip_default=lambda th: boundary_stream())
    run_cmd("PLR_DRAG_CALIBRATE", START=1)
    result = plugin.last_drag_calibrate
    tested = result["tested"]
    # Strictly decreasing, starts at the MOST sensitive knob.
    assert tested[0] == pytest.approx(100.0)
    assert all(a > b for a, b in zip(tested, tested[1:]))
    # Accepted is the last (first survivor); it is <= the 60 boundary and
    # the highest step-landing that survives.
    accepted = result["accepted_knob"]
    assert accepted == tested[-1]
    assert accepted <= 60.0
    # And the step immediately before it was still above the boundary
    # (the sweep did not stop early / test lower than necessary).
    assert tested[-2] > 60.0
    # Deterministic landing for this stream: 100 -> 85 -> 70 -> 56.
    assert tested == [100.0, 85.0, 70.0, 56.0]
    assert accepted == pytest.approx(56.0)


# ---------------------------------------------------------------------
# Verification early-exit


def test_verify_early_exits_on_first_false_contact(calib_setup, run_cmd):
    """Screening survives at knob 100, but verify pass 2 of 6 false-
    triggers: exactly 2 verify passes run (not all 6), then the sweep
    steps down and the next knob is accepted."""
    contact = sf.with_contact(sf.quiet(seed=44), amplitude=500.0)
    clean_s = sf.quiet(seed=41)
    script = (
        [clean_s, clean_s, clean_s]  # screen @100 (3 clean)
        + [clean_s, contact]  # verify @100: pass1 clean, pass2 contact -> exit
        + [clean_s, clean_s, clean_s]  # screen @90 (3 clean)
        + [clean_s] * 6  # verify @90 (6 clean) -> accept
    )
    plugin, toolhead, chip = calib_setup(chip_script=script)
    run_cmd("PLR_DRAG_CALIBRATE", START=1)
    result = plugin.last_drag_calibrate
    assert result["tested"] == [100.0, 90.0]
    assert result["accepted_knob"] == pytest.approx(90.0)
    # 3 + 2 (early exit!) + 3 + 6 = 14.  Without the early exit, verify
    # @100 would run all 6 -> 3 + 6 + 3 + 6 = 18.
    assert len(chip.clients) == 14


# ---------------------------------------------------------------------
# Acceptance staging and headroom


def test_accept_stages_sensitivity_with_margin(calib_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    run_cmd("PLR_DRAG_CALIBRATE", START=1, MARGIN=5)
    pending = fake_printer.lookup_object("configfile").pending.get("plr", {})
    # accepted 100 - margin 5 = 95.
    assert pending["drag_sensitivity"] == tunables.format_value(95.0)
    assert plugin.is_pending_save("drag_sensitivity")
    assert plugin.tunables["drag_sensitivity"] == pytest.approx(95.0)
    assert plugin.last_drag_calibrate["recommended"] == pytest.approx(95.0)


def test_margin_floors_at_zero(calib_setup, run_cmd):
    """A large margin cannot push the recommendation below 0."""
    plugin, toolhead, chip = calib_setup(chip_default=lambda th: boundary_stream())
    run_cmd("PLR_DRAG_CALIBRATE", START=1, MARGIN=50)
    # accepted 56, 56 - 50 = 6.
    assert plugin.last_drag_calibrate["recommended"] == pytest.approx(6.0)


def test_report_has_headroom_and_save_config_hint(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    gcode = run_cmd("PLR_DRAG_CALIBRATE", START=1)
    report = gcode.responses[-1]
    assert "headroom" in report
    assert "SAVE_CONFIG" in report
    assert "accepted knob 100" in report


# ---------------------------------------------------------------------
# Exhaustion is not an exception


def test_exhaustion_is_info_not_error(calib_setup, run_cmd):
    """Every knob false-triggers (even the least sensitive): an info log
    with a copy-pasteable retry, no exception, nothing staged."""
    plugin, toolhead, chip = calib_setup(chip_default=hard_contact)
    gcode = run_cmd("PLR_DRAG_CALIBRATE", START=1)  # must NOT raise
    report = gcode.responses[-1]
    assert "no knob" in report
    assert "PLR_NOISE_TEST" in report
    assert "PLR_DRAG_CALIBRATE START=1" in report  # the retry line
    assert plugin.last_drag_calibrate is None
    assert not plugin.is_pending_save("drag_sensitivity")


def test_exhaustion_retry_line_is_parseable(calib_setup, run_cmd, fake_printer):
    """The retry command round-trips through the g-code parser."""
    plugin, toolhead, chip = calib_setup(chip_default=hard_contact)
    gcode = run_cmd("PLR_DRAG_CALIBRATE", START=1, SPEED=25)
    report = gcode.responses[-1]
    retry_line = [
        ln for ln in report.splitlines() if "PLR_DRAG_CALIBRATE START=1" in ln
    ]
    assert retry_line
    line = retry_line[0].strip()
    name, *pairs = line.split()
    assert name == "PLR_DRAG_CALIBRATE"
    params = dict(p.split("=", 1) for p in pairs)
    gcmd = fake_printer.lookup_object("gcode").create_gcode_command(name, line, params)
    assert gcmd.get_int("START") == 1
    assert gcmd.get_float("SPEED") == pytest.approx(25.0)


# ---------------------------------------------------------------------
# Degenerate captures abort (cannot calibrate on junk)


def test_no_data_pass_aborts(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_script=[None])
    with pytest.raises(fake_klippy.FakeCommandError, match="measured no data"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert plugin.last_drag_calibrate is None


def test_invalid_pass_aborts(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_script=[sf.quiet(n=100)])
    with pytest.raises(fake_klippy.FakeCommandError, match="uncalibratable pass"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert plugin.last_drag_calibrate is None


# ---------------------------------------------------------------------
# Consent / plan mode


def test_without_start_prints_plan_and_moves_nothing(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    gcode = run_cmd("PLR_DRAG_CALIBRATE")
    assert "no motion yet" in gcode.responses[-1]
    assert "never descends" in gcode.responses[-1]
    assert "START=1" in gcode.responses[-1]
    assert toolhead.moves == []
    assert chip.clients == []


# ---------------------------------------------------------------------
# Invocation gates


def test_refused_while_printing(calib_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    fake_printer.objects["idle_timeout"].state = "Printing"
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert toolhead.moves == []
    assert plugin.last_drag_calibrate is None


def test_refused_unless_homed(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    toolhead.homed_axes = "xy"
    with pytest.raises(fake_klippy.FakeCommandError, match="must be homed"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert toolhead.moves == []


def test_refused_without_noise_floor(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(
        chip_default=clean, options={"noise_floor_rms": None}
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="PLR_NOISE_TEST"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert toolhead.moves == []


def test_refused_when_chip_missing(calib_setup, run_cmd):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    with pytest.raises(fake_klippy.FakeCommandError, match="chip sections found"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1, CHIP="mpu9250")
    assert toolhead.moves == []


def test_refused_without_finite_z_floor(fake_printer, plr_config, run_cmd):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, START_Z, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(fake_printer, default=clean)
    fake_printer.add_object("adxl345", chip)
    plr.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345",
                "noise_floor_rms": NOISE_FLOOR,
            },
            sections={"stepper_z": {"step_pin": "PF11"}},  # no position_min
        )
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="Z floor"):
        run_cmd("PLR_DRAG_CALIBRATE", START=1)
    assert toolhead.moves == []


@pytest.mark.parametrize(
    ("param", "value", "text"),
    [
        ("SPEED", "0", "must be above"),
        ("SPEED", "101", "maximum"),
        ("SCREEN_PASSES", "0", "minimum"),
        ("VERIFY_PASSES", "0", "minimum"),
        ("MARGIN", "-1", "minimum"),
        ("MARGIN", "51", "maximum"),
    ],
)
def test_out_of_range_args_refused(calib_setup, run_cmd, param, value, text):
    plugin, toolhead, chip = calib_setup(chip_default=clean)
    with pytest.raises(fake_klippy.FakeCommandError, match=text):
        run_cmd("PLR_DRAG_CALIBRATE", START=1, **{param: value})
    assert toolhead.moves == []
