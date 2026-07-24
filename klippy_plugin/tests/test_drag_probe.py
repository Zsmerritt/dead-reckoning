"""Tests for the PLR_DRAG_PROBE staircase orchestration.

The fakes supply the SCRIPT of what samples arrive per pass (glue); the
classification of those samples is always the real classifier math.
Safety invariants are asserted over the recorded manual_move list —
including under hostile scripts that never report contact.
"""

import math

import fake_klippy
import pytest
import stream_fixtures as sf

import plr
from plr import classifier, drag_probe

# Noise floor consistent with sf.quiet(noise=5.0): stream RMS ~= 8.66.
NOISE_FLOOR = "8.66"

# Geometry used by most tests: start Z 1.0, [stepper_z] position_min 0,
# Z_STEP 0.2 -> floor 0.2, iteration bound ceil(0.8/0.2) = 4 passes at
# Z 1.0 / 0.8 / 0.6 / 0.4.
START_Z = 1.0
Z_STEP = 0.2
FLOOR = 0.2
BOUND = 4


def quiet_stream(toolhead):
    return sf.quiet(seed=11)


def contact_stream(toolhead):
    return sf.with_contact(sf.quiet(seed=12), amplitude=500.0)


def contact_below(z_trigger):
    """Script factory: contact whenever the pass runs at Z <= z_trigger."""

    def script(toolhead):
        if toolhead.position[2] <= z_trigger:
            return contact_stream(toolhead)
        return quiet_stream(toolhead)

    return script


@pytest.fixture
def drag_setup(fake_printer, plr_config):
    """Plugin + homed toolhead + scripted chip for drag-probe tests."""

    def build(
        chip_script=None,
        chip_default=None,
        start_z=START_Z,
        options=None,
        toolhead_position_min=None,
        chip_name="adxl345",
    ):
        toolhead = fake_klippy.FakeToolhead(
            position=(150.0, 150.0, start_z, 0.0),
            position_min=toolhead_position_min,
        )
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
            "drag_z_step": str(Z_STEP),
        }
        merged.update(options or {})
        # A None value removes the key (e.g. "no noise floor on record").
        merged = {k: v for k, v in merged.items() if v is not None}
        plugin = plr.load_config(
            plr_config(
                options=merged,
                sections={"stepper_z": {"step_pin": "PF11", "position_min": "0"}},
            )
        )
        return plugin, toolhead, chip

    return build


def lateral_moves(moves):
    """Moves that change XY (pass segments)."""
    return [m for m in moves if m[0][0] is not None]


def z_moves(moves):
    """Z-only moves (descents, lifts, restores)."""
    return [m for m in moves if m[0][0] is None and m[0][2] is not None]


def commanded_z_values(moves):
    return [m[0][2] for m in moves if len(m[0]) > 2 and m[0][2] is not None]


# ---------------------------------------------------------------------
# Pure helpers


def test_travel_seconds_basic():
    assert drag_probe.travel_seconds(30.0, 5.0) == pytest.approx(6.0)


def test_zero_distance_is_instant():
    assert drag_probe.travel_seconds(0.0, 5.0) == 0.0


@pytest.mark.parametrize("speed", [0.0, -1.0])
def test_nonpositive_speed_rejected(speed):
    with pytest.raises(ValueError, match="must be positive"):
        drag_probe.travel_seconds(10.0, speed)


def test_negative_distance_rejected():
    with pytest.raises(ValueError, match="must not be negative"):
        drag_probe.travel_seconds(-1.0, 5.0)


@pytest.mark.parametrize(
    ("start", "floor", "step", "expected"),
    [
        (1.0, 0.2, 0.2, 4),
        (5.0, -1.95, 0.05, 139),
        (1.0, 0.9, 0.2, 1),  # partial step still gets one pass
        (0.30000000000000004, 0.2, 0.05, 3),
    ],
)
def test_iteration_bound_values(start, floor, step, expected):
    assert drag_probe.iteration_bound(start, floor, step) == expected


def test_iteration_bound_no_envelope_rejected():
    with pytest.raises(ValueError, match="no probing envelope"):
        drag_probe.iteration_bound(0.2, 0.2, 0.05)


def test_iteration_bound_bad_step_rejected():
    with pytest.raises(ValueError, match="must be positive"):
        drag_probe.iteration_bound(1.0, 0.0, 0.0)


# ---------------------------------------------------------------------
# Staircase success path


def test_staircase_descends_z_step_per_clean_pass_then_stops(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE")
    descents = [m for m in z_moves(toolhead.moves) if m[0][2] < START_Z]
    # Two clean passes (1.0, 0.8) -> two descents (0.8, 0.6); contact at
    # 0.6 stops the staircase: no descent below 0.6 ever commanded.
    assert [m[0][2] for m in descents] == [pytest.approx(0.8), pytest.approx(0.6)]
    assert min(commanded_z_values(toolhead.moves)) >= FLOOR
    # Descents run at the probe_speed tunable (default 1.5).
    assert all(m[1] == pytest.approx(1.5) for m in descents)


def test_trigger_z_is_last_clean_pass(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE")
    result = plugin.last_drag_result
    assert result is not None
    assert result["trigger_z"] == pytest.approx(0.8)  # last CLEAN pass
    assert result["passes"] == 3  # two clean + the contact pass
    assert 0.0 <= result["confidence"] <= 1.0
    assert plugin.last_drag_error is None


def test_contact_lifts_clearance_bounded_by_start(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE")
    # Contact at 0.6: lift to min(start, 0.6 + 2*0.2) = 1.0 = start.
    assert toolhead.position[2] == pytest.approx(1.0)
    lift = z_moves(toolhead.moves)[-1]
    assert lift[0][2] == pytest.approx(1.0)
    assert lift[1] == pytest.approx(drag_probe.LIFT_SPEED)


def test_deep_contact_clearance_is_two_steps(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.41))
    run_cmd("PLR_DRAG_PROBE")
    # Contact at 0.4 -> clearance min(1.0, 0.4 + 0.4) = 0.8.
    assert toolhead.position[2] == pytest.approx(0.8)
    assert plugin.last_drag_result["trigger_z"] == pytest.approx(0.6)


def test_pass_geometry_centered_on_start_xy(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE")
    seg = drag_probe.DEFAULT_PASS_LENGTH / 4.0
    first_pass = lateral_moves(toolhead.moves)[:3]
    assert [m[0][:2] for m in first_pass] == [
        [150.0 + seg, 150.0],
        [150.0 - seg, 150.0],
        [150.0, 150.0],
    ]
    # Lateral speed is the drag_speed tunable default.
    assert all(m[1] == pytest.approx(20.0) for m in first_pass)
    # Every pass runs at a FIXED Z: lateral moves never carry a Z.
    assert all(m[0][2] is None for m in lateral_moves(toolhead.moves))


def test_get_status_exposes_result(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE")
    status = plugin.get_status(100.0)
    assert status["last_drag_result"] == plugin.last_drag_result
    assert status["last_drag_error"] is None


# ---------------------------------------------------------------------
# Hostile script: never contact -> iteration bound + floor honored


def test_all_clean_hits_iteration_bound_and_restores(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=quiet_stream)
    with pytest.raises(fake_klippy.FakeCommandError, match="no contact within"):
        run_cmd("PLR_DRAG_PROBE")
    # Exactly BOUND passes ran (one internal client each).
    assert len(chip.clients) == BOUND
    assert len(lateral_moves(toolhead.moves)) == BOUND * 3
    # THE safety assertion: no commanded Z ever below the floor, even
    # though the script begged for descent forever.
    assert min(commanded_z_values(toolhead.moves)) >= FLOOR
    # Z restored to the start height.
    assert toolhead.position[2] == pytest.approx(START_Z)
    assert plugin.last_drag_result is None
    assert "no contact within envelope" in plugin.last_drag_error
    assert "SENSITIVITY" in plugin.last_drag_error


def test_klippy_backstop_never_the_mechanism(drag_setup, run_cmd):
    """With the fake kinematic limit set exactly at the plugin's floor,
    a hostile all-clean run must end via the plugin's own bound/floor
    logic — never via the toolhead's 'Move out of range' backstop."""
    plugin, toolhead, chip = drag_setup(
        chip_default=quiet_stream, toolhead_position_min=FLOOR
    )
    with pytest.raises(fake_klippy.FakeCommandError) as excinfo:
        run_cmd("PLR_DRAG_PROBE")
    assert "Move out of range" not in str(excinfo.value)
    assert "no contact within envelope" in str(excinfo.value)


def test_bound_computed_up_front_matches_iteration_bound(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(
        chip_default=quiet_stream, start_z=0.5
    )  # floor 0.2 -> bound ceil(0.3/0.2) = 2
    with pytest.raises(fake_klippy.FakeCommandError):
        run_cmd("PLR_DRAG_PROBE")
    assert len(chip.clients) == drag_probe.iteration_bound(0.5, FLOOR, Z_STEP) == 2


# ---------------------------------------------------------------------
# Abort paths


def test_invalid_pass_aborts_and_restores(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(
        chip_script=[quiet_stream, sf.quiet(n=100, seed=13)],
        chip_default=quiet_stream,
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="too_few_samples"):
        run_cmd("PLR_DRAG_PROBE")
    # Aborted on pass 2 (Z 0.8): no descent past it, Z restored.
    assert len(chip.clients) == 2
    assert toolhead.position[2] == pytest.approx(START_Z)
    assert plugin.last_drag_result is None
    assert "invalid pass" in plugin.last_drag_error


def test_no_data_pass_aborts(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_script=[None])
    with pytest.raises(fake_klippy.FakeCommandError, match="measured no data"):
        run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_result is None
    assert toolhead.position[2] == pytest.approx(START_Z)


def test_first_pass_contact_is_an_error_not_a_result(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_script=[contact_stream])
    with pytest.raises(fake_klippy.FakeCommandError, match="first pass"):
        run_cmd("PLR_DRAG_PROBE")
    # No clean Z exists, so no trigger_z may be reported.
    assert plugin.last_drag_result is None
    assert "start higher" in plugin.last_drag_error
    # Never descended.
    assert z_moves(toolhead.moves) == []


# ---------------------------------------------------------------------
# Invocation gates


def test_refused_while_printing(drag_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = drag_setup()
    fake_printer.objects["idle_timeout"].state = "Printing"
    with pytest.raises(fake_klippy.FakeCommandError, match="print is active"):
        run_cmd("PLR_DRAG_PROBE")
    assert toolhead.moves == []
    assert plugin.last_drag_error is not None


def test_refused_unless_fully_homed(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup()
    toolhead.homed_axes = "xy"
    with pytest.raises(fake_klippy.FakeCommandError, match="must be homed"):
        run_cmd("PLR_DRAG_PROBE")
    assert toolhead.moves == []


def test_refused_without_noise_floor(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(options={"noise_floor_rms": None})
    with pytest.raises(fake_klippy.FakeCommandError, match="PLR_NOISE_TEST"):
        run_cmd("PLR_DRAG_PROBE")
    assert toolhead.moves == []
    assert "PLR_NOISE_TEST" in plugin.last_drag_error


def test_refused_when_chip_missing_lists_found_chips(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup()
    with pytest.raises(
        fake_klippy.FakeCommandError, match="chip sections found: adxl345"
    ):
        run_cmd("PLR_DRAG_PROBE", CHIP="adxl345 head")
    assert toolhead.moves == []


def test_refused_when_object_is_not_an_accelerometer(drag_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = drag_setup()
    fake_printer.add_object("adxl345 brick", object())
    with pytest.raises(fake_klippy.FakeCommandError, match="not an accelerometer"):
        run_cmd("PLR_DRAG_PROBE", CHIP="adxl345 brick")


def test_refused_at_or_below_floor(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(start_z=0.15)
    with pytest.raises(fake_klippy.FakeCommandError, match="at/below the Z floor"):
        run_cmd("PLR_DRAG_PROBE")
    assert toolhead.moves == []


@pytest.mark.parametrize(
    ("param", "value", "text"),
    [
        ("SPEED", "0", "must be above"),
        ("SPEED", "101", "maximum"),
        ("Z_STEP", "0", "must be above"),
        ("Z_STEP", "0.3", "maximum"),
        ("SENSITIVITY", "-1", "minimum"),
        ("SENSITIVITY", "101", "maximum"),
        ("PASS_LENGTH", "1", "minimum"),
        ("PASS_LENGTH", "25", "maximum"),
    ],
)
def test_out_of_range_args_refused(drag_setup, run_cmd, param, value, text):
    plugin, toolhead, chip = drag_setup()
    with pytest.raises(fake_klippy.FakeCommandError, match=text):
        run_cmd("PLR_DRAG_PROBE", **{param: value})
    assert toolhead.moves == []
    assert plugin.last_drag_result is None
    assert plugin.last_drag_error is not None


# ---------------------------------------------------------------------
# CHIP resolution incl. spaced (quoted) section names


def test_spaced_chip_name_resolves(fake_printer, plr_config, run_cmd):
    """klippy's extended-command parser (gcode.py:266-281, shlex posix)
    hands CHIP="adxl345 bed" to the handler as the single value
    'adxl345 bed'; resolution must accept it as-is."""
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, START_Z, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(fake_printer, default=contact_below(0.61))
    fake_printer.add_object("adxl345 bed", chip)
    plugin = plr.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345 bed",
                "noise_floor_rms": NOISE_FLOOR,
                "drag_z_step": str(Z_STEP),
            },
            sections={
                "stepper_z": {"step_pin": "PF11", "position_min": "0"},
                "adxl345 bed": {"cs_pin": "PB1"},
            },
        )
    )
    run_cmd("PLR_DRAG_PROBE", CHIP="adxl345 bed")
    assert plugin.last_drag_result is not None
    assert len(chip.clients) == 3


def test_chip_arg_overrides_configured_chip(drag_setup, run_cmd, fake_printer):
    plugin, toolhead, default_chip = drag_setup()
    other = fake_klippy.FakeAccelChip(fake_printer, default=contact_below(0.61))
    fake_printer.add_object("lis2dw", other)
    run_cmd("PLR_DRAG_PROBE", CHIP="lis2dw")
    assert default_chip.clients == []
    assert len(other.clients) == 3


# ---------------------------------------------------------------------
# Argument override precedence over [plr] tunables


def test_speed_and_pass_length_args_override_tunables(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE", SPEED="33", PASS_LENGTH="4")
    first_pass = lateral_moves(toolhead.moves)[:3]
    assert all(m[1] == pytest.approx(33.0) for m in first_pass)
    assert first_pass[0][0][0] == pytest.approx(150.0 + 1.0)  # seg = 4/4


def test_z_step_arg_overrides_tunable(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.85))
    run_cmd("PLR_DRAG_PROBE", Z_STEP="0.1")
    descents = [m for m in z_moves(toolhead.moves) if m[0][2] < START_Z]
    # First descent is exactly one overridden step below start.
    assert descents[0][0][2] == pytest.approx(START_Z - 0.1)
    assert plugin.last_drag_result["trigger_z"] == pytest.approx(0.9)


def test_sensitivity_arg_overrides_tunable(drag_setup, run_cmd, fake_printer):
    """A faint burst (amplitude 30) is below the default knob-30
    threshold but above the knob-100 threshold: the arg must win."""
    faint = sf.with_contact(sf.quiet(seed=14), amplitude=30.0)
    # Sanity-check the premise with the real classifier.
    floor = float(NOISE_FLOOR)
    assert classifier.classify_pass(faint, floor, 30.0).contact is False
    assert classifier.classify_pass(faint, floor, 100.0).contact is True

    plugin, toolhead, chip = drag_setup(
        chip_script=[quiet_stream, faint], chip_default=quiet_stream
    )
    run_cmd("PLR_DRAG_PROBE", SENSITIVITY="100")
    assert plugin.last_drag_result is not None
    assert plugin.last_drag_result["passes"] == 2
    assert plugin.last_drag_result["trigger_z"] == pytest.approx(START_Z)


def test_default_sensitivity_misses_faint_contact(drag_setup, run_cmd):
    faint = sf.with_contact(sf.quiet(seed=14), amplitude=30.0)
    plugin, toolhead, chip = drag_setup(
        chip_script=[quiet_stream, faint], chip_default=quiet_stream
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="no contact"):
        run_cmd("PLR_DRAG_PROBE")  # tunable default sensitivity 30


# ---------------------------------------------------------------------
# Misc orchestration details


def test_settle_before_every_pass(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    run_cmd("PLR_DRAG_PROBE")
    settles = [d for d in toolhead.dwells if d == drag_probe.SETTLE_SECONDS]
    assert len(settles) == 3  # one per pass


def test_clients_all_finished(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=quiet_stream)
    with pytest.raises(fake_klippy.FakeCommandError):
        run_cmd("PLR_DRAG_PROBE")
    assert all(c.finished for c in chip.clients)


def test_missing_z_floor_refused(fake_printer, plr_config, run_cmd):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, START_Z, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(fake_printer, default=quiet_stream)
    fake_printer.add_object("adxl345", chip)
    plugin = plr.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345",
                "noise_floor_rms": NOISE_FLOOR,
            },
            sections={"stepper_z": {"step_pin": "PF11"}},
        )
    )
    assert plugin.z_position_min is None
    with pytest.raises(fake_klippy.FakeCommandError, match="Z floor"):
        run_cmd("PLR_DRAG_PROBE")
    assert toolhead.moves == []


def test_success_clears_previous_error(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    plugin.last_drag_error = "stale"
    run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_error is None


def test_failure_clears_previous_result(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=quiet_stream)
    plugin.last_drag_result = {"trigger_z": 9.9, "passes": 1, "confidence": 1.0}
    with pytest.raises(fake_klippy.FakeCommandError):
        run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_result is None


def test_report_mentions_trigger_band(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=contact_below(0.61))
    gcode = run_cmd("PLR_DRAG_PROBE")
    report = gcode.responses[-1]
    assert "trigger_z = 0.800" in report
    assert "(0.600, 0.800]" in report


def test_iteration_bound_float_edges_never_below_floor(drag_setup, run_cmd):
    """Float-unfriendly geometry: bound math and the per-descent floor
    check must agree that nothing goes below the floor."""
    plugin, toolhead, chip = drag_setup(
        chip_default=quiet_stream,
        start_z=1.01,
        options={"drag_z_step": "0.19"},
    )
    with pytest.raises(fake_klippy.FakeCommandError, match="no contact"):
        run_cmd("PLR_DRAG_PROBE", Z_STEP="0.19")
    floor = 0.0 + 0.19
    zs = commanded_z_values(toolhead.moves)
    assert all(z >= floor - 1e-12 for z in zs)


def test_math_isfinite_guard_against_inf_floor(fake_printer, plr_config, run_cmd):
    toolhead = fake_klippy.FakeToolhead(position=(150.0, 150.0, START_Z, 0.0))
    fake_printer.add_object("toolhead", toolhead)
    fake_printer.add_object("idle_timeout", fake_klippy.FakeIdleTimeout())
    chip = fake_klippy.FakeAccelChip(fake_printer, default=quiet_stream)
    fake_printer.add_object("adxl345", chip)
    plugin = plr.load_config(
        plr_config(
            options={
                "probe_method": "adxl_drag",
                "accel_chip": "adxl345",
                "noise_floor_rms": NOISE_FLOOR,
            },
            sections={"stepper_z": {"step_pin": "PF11", "position_min": "-inf"}},
        )
    )
    assert not math.isfinite(plugin.z_position_min)
    with pytest.raises(fake_klippy.FakeCommandError, match="finite Z floor"):
        run_cmd("PLR_DRAG_PROBE")
    assert toolhead.moves == []
