"""Tests for the PLR_DRAG_PROBE staircase orchestration.

The fakes supply the SCRIPT of what samples arrive per pass (glue); the
classification of those samples is always the real classifier math.
Safety invariants are asserted over the recorded manual_move list —
including under hostile scripts that never report contact.
"""

import math
import shlex

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


@pytest.mark.parametrize(
    "raw",
    [
        'CHIP="adxl345 bed" SPEED=25',  # double-quoted (plrd emits this)
        "CHIP='adxl345 bed' SPEED=25",  # single-quoted
    ],
)
def test_quoted_chip_through_extended_params_parse(
    fake_printer, plr_config, run_cmd, raw
):
    """Fidelity test of the quoted-CHIP contract, end to end.

    Replicates klippy's extended-parameter parse EXACTLY as
    gcode.py:266-281 performs it (shlex posix + whitespace_split,
    split each token at the first '=', uppercase keys, values keep
    case and embedded spaces) over the raw quoted command line the
    Rust plan emits, then dispatches the resulting params.  If klippy
    changes this contract, this test's premise assertions break first.
    """
    s = shlex.shlex(raw, posix=True)
    s.whitespace_split = True
    s.commenters = "#;"
    eparams = [earg.split("=", 1) for earg in s]
    params = {k.upper(): v for k, v in eparams}
    # The premise proved against klippy's parser: quotes stripped, the
    # embedded space and value case preserved.
    assert params == {"CHIP": "adxl345 bed", "SPEED": "25"}

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
    run_cmd("PLR_DRAG_PROBE", **params)
    assert plugin.last_drag_result is not None
    assert len(chip.clients) == 3
    # SPEED came through the same parse.
    assert all(m[1] == pytest.approx(25.0) for m in lateral_moves(toolhead.moves))


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


# =====================================================================
# Staircase hardening
# =====================================================================

# ---------------------------------------------------------------------
# Pure helpers: coverage bracketing + temperature widening


def test_check_coverage_ok_window():
    samples = [(100.0 + i / 3200.0, 1.0, 2.0, 3.0) for i in range(1024)]
    # motion [100.0, 100.30]; samples span [100.0, 100.319] -> covered.
    assert drag_probe.check_coverage(samples, 100.0, 100.30, 0.15) is None


def test_check_coverage_late_start():
    # First sample lands well after motion start + grace.
    samples = [(100.5 + i / 3200.0, 1.0, 2.0, 3.0) for i in range(1024)]
    invalid = drag_probe.check_coverage(samples, 100.0, 100.30, 0.15)
    assert isinstance(invalid, classifier.PassInvalid)
    assert invalid.reason == classifier.INVALID_COVERAGE
    assert "began late" in invalid.detail


def test_check_coverage_short_end():
    # Last sample lands well before motion end - grace.
    samples = [(100.0 + i / 3200.0, 1.0, 2.0, 3.0) for i in range(256)]
    invalid = drag_probe.check_coverage(samples, 100.0, 100.60, 0.15)
    assert isinstance(invalid, classifier.PassInvalid)
    assert invalid.reason == classifier.INVALID_COVERAGE
    assert "ran short" in invalid.detail


@pytest.mark.parametrize(
    ("staged", "current", "expected"),
    [
        (25.0, 25.0, 0.0),  # exact
        (25.0, 40.0, 0.0),  # deviation 15 == band edge -> no widening
        (25.0, 45.0, 0.10),  # deviation 20 -> 5 past band -> +10%
        (25.0, 65.0, 0.50),  # deviation 40 -> capped at +50%
        (25.0, 200.0, 0.50),  # far beyond -> still capped
        (60.0, 20.0, 0.50),  # colder machine widens too (never narrows)
    ],
)
def test_temp_widen_factor_formula(staged, current, expected):
    assert drag_probe.temp_widen_factor(staged, current) == pytest.approx(expected)


def test_temp_widen_factor_never_negative():
    # Across a sweep of deviations the factor is always >= 0 (never narrows).
    for current in range(-40, 200, 7):
        assert drag_probe.temp_widen_factor(25.0, float(current)) >= 0.0


def test_read_temp_absent_sensor_is_none(drag_setup):
    plugin, toolhead, chip = drag_setup()
    assert drag_probe.read_temp(plugin, None) is None
    assert drag_probe.read_temp(plugin, "temperature_sensor ghost") is None


def test_read_temp_reads_sensor(drag_setup, fake_printer):
    plugin, toolhead, chip = drag_setup()
    fake_printer.add_object("temperature_sensor cham", fake_klippy.FakeTempSensor(42.5))
    assert drag_probe.read_temp(plugin, "temperature_sensor cham") == pytest.approx(
        42.5
    )


def test_read_temp_no_reading_is_none(drag_setup, fake_printer):
    plugin, toolhead, chip = drag_setup()
    fake_printer.add_object("temperature_sensor cham", fake_klippy.FakeTempSensor(None))
    assert drag_probe.read_temp(plugin, "temperature_sensor cham") is None


def test_read_temp_non_numeric_is_none(drag_setup, fake_printer):
    plugin, toolhead, chip = drag_setup()
    fake_printer.add_object(
        "temperature_sensor cham", fake_klippy.FakeTempSensor("warm")
    )
    assert drag_probe.read_temp(plugin, "temperature_sensor cham") is None


def test_read_temp_non_finite_is_none(drag_setup, fake_printer):
    plugin, toolhead, chip = drag_setup()
    fake_printer.add_object(
        "temperature_sensor cham", fake_klippy.FakeTempSensor(float("inf"))
    )
    assert drag_probe.read_temp(plugin, "temperature_sensor cham") is None


def test_read_temp_raising_sensor_is_none(drag_setup, fake_printer):
    class Raising:
        def get_status(self, eventtime):
            raise RuntimeError("sensor not ready")

    plugin, toolhead, chip = drag_setup()
    fake_printer.add_object("temperature_sensor cham", Raising())
    assert drag_probe.read_temp(plugin, "temperature_sensor cham") is None


# ---------------------------------------------------------------------
# Impossible-result branch


def decreasing_signal_script():
    """Clean passes whose ratio-to-threshold falls monotonically while
    staying >= 50% of threshold (peaks 35.5 -> 31.3 -> 27.0 at knob 30,
    ratios ~0.78 > 0.68 > 0.59): physically implausible."""
    return [sf.with_contact(sf.quiet(seed=60), a) for a in (30.0, 26.0, 22.0)]


def test_receding_signal_aborts_typed_and_restores(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(
        chip_script=decreasing_signal_script(), chip_default=quiet_stream
    )
    with pytest.raises(
        fake_klippy.FakeCommandError, match=drag_probe.ABORT_IMPLAUSIBLE
    ):
        run_cmd("PLR_DRAG_PROBE")
    # Aborts as soon as the 3-pass receding run is complete; never runs
    # the whole envelope.
    assert len(chip.clients) == drag_probe.IMPLAUSIBLE_RUN
    assert toolhead.position[2] == pytest.approx(START_Z)
    assert plugin.last_drag_result is None
    assert "receding" in plugin.last_drag_error


def test_increasing_signal_not_flagged_implausible(drag_setup, run_cmd):
    """Regression: a normal rising staircase (signal grows as we approach)
    is never mistaken for the implausible receding case."""
    rising = [sf.with_contact(sf.quiet(seed=60), a) for a in (18.0, 22.0, 26.0)]
    plugin, toolhead, chip = drag_setup(chip_script=rising, chip_default=contact_stream)
    run_cmd("PLR_DRAG_PROBE")
    # Three rising clean passes, then contact on the fourth: a normal
    # result, not an implausible abort.
    assert plugin.last_drag_result is not None
    assert plugin.last_drag_error is None
    assert plugin.last_drag_result["passes"] == 4


# ---------------------------------------------------------------------
# Data-coverage bracketing (bound via PassInvalid)


def test_short_capture_fails_coverage(drag_setup, run_cmd):
    # A valid-but-short capture (400 samples ~= 0.125 s) does not span the
    # ~0.4 s pass motion -> coverage_gap PassInvalid abort.
    plugin, toolhead, chip = drag_setup(chip_script=[sf.quiet(n=400, seed=13)])
    with pytest.raises(fake_klippy.FakeCommandError, match="coverage_gap"):
        run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_result is None
    assert toolhead.position[2] == pytest.approx(START_Z)
    assert "ran short" in plugin.last_drag_error


# ---------------------------------------------------------------------
# Three independent bounds


def test_iteration_bound_is_typed(drag_setup, run_cmd):
    plugin, toolhead, chip = drag_setup(chip_default=quiet_stream)
    with pytest.raises(fake_klippy.FakeCommandError, match=drag_probe.ABORT_ENVELOPE):
        run_cmd("PLR_DRAG_PROBE")
    assert "no contact within envelope" in plugin.last_drag_error


def test_wall_clock_budget_aborts(drag_setup, run_cmd, fake_printer):
    # Drive the reactor clock forward 20 s per read: with MAX_SECONDS=30
    # the budget trips on the second pass, before the (large) iteration
    # bound or the stall detector.
    plugin, toolhead, chip = drag_setup(chip_default=quiet_stream, start_z=5.0)
    fake_printer.get_reactor().auto_advance = 20.0
    with pytest.raises(
        fake_klippy.FakeCommandError, match=drag_probe.ABORT_TIME_BUDGET
    ):
        run_cmd("PLR_DRAG_PROBE", MAX_SECONDS=30, STALL_PASSES=100)
    assert plugin.last_drag_result is None
    assert toolhead.position[2] == pytest.approx(5.0)
    assert "wall-clock budget" in plugin.last_drag_error
    # Tripped early, long before the ceil(4.8/0.2)=24 iteration bound.
    assert len(chip.clients) < 5


def test_stall_warns_at_half_then_aborts(drag_setup, run_cmd, fake_printer):
    # Identical flat clean passes with plenty of travel: stall detector
    # warns at half the budget and aborts at the full budget, before the
    # iteration bound (24) and the wall-clock budget (default 120 s).
    plugin, toolhead, chip = drag_setup(chip_default=quiet_stream, start_z=5.0)
    with pytest.raises(fake_klippy.FakeCommandError, match=drag_probe.ABORT_STALL):
        run_cmd("PLR_DRAG_PROBE", STALL_PASSES=8)
    # Warn-at-half asserted: exactly one no-progress warning before abort.
    responses = fake_printer.lookup_object("gcode").responses
    warnings = [r for r in responses if "no-progress warning" in r]
    assert len(warnings) == 1
    assert drag_probe.ABORT_STALL in warnings[0]
    assert plugin.last_drag_result is None
    assert toolhead.position[2] == pytest.approx(5.0)
    # Flat run of 8 needs 8 x z_step of descent to abort -> the 9th pass.
    assert len(chip.clients) == 9


# ---------------------------------------------------------------------
# Temperature covariate


def _temp_setup(drag_setup, fake_printer, sensor_temp, staged_temp="25.0"):
    options = {"noise_floor_temp_sensor": "temperature_sensor cham"}
    if staged_temp is not None:
        options["noise_floor_temp"] = staged_temp
    plugin, toolhead, chip = drag_setup(
        chip_script=[quiet_stream, sf.with_contact(sf.quiet(seed=70), 44.0)],
        chip_default=quiet_stream,
        options=options,
    )
    if sensor_temp is not None:
        fake_printer.add_object(
            "temperature_sensor cham", fake_klippy.FakeTempSensor(sensor_temp)
        )
    return plugin, toolhead, chip


def test_temp_deviation_widens_threshold_numeric(drag_setup, run_cmd, fake_printer):
    plugin, toolhead, chip = _temp_setup(drag_setup, fake_printer, sensor_temp=65.0)
    # Widening makes the faint pass-2 contact read clean, so the probe
    # finds no contact -- but the widened-threshold report is emitted
    # before that, and is what we assert on here.
    with pytest.raises(fake_klippy.FakeCommandError):
        run_cmd("PLR_DRAG_PROBE")  # staged 25, current 65 -> +50%
    gresponses = fake_printer.lookup_object("gcode").responses
    responses = "\n".join(gresponses)
    # The widened threshold is exactly 1.5x the base: base 45.708 -> 68.56.
    base = float(NOISE_FLOOR) * classifier.multiplier(30.0)
    assert "%.2f" % (base * 1.5,) in responses
    # Warned once, and the warning names the +50% cap.
    warns = [r for r in gresponses if "widening the threshold" in r]
    assert len(warns) == 1
    assert "+50%" in warns[0]


def test_temp_widening_flips_contact_to_clean(drag_setup, run_cmd, fake_printer):
    """The faint pass-2 contact (peak ~50) is above the base threshold
    (45.7) but below the widened one (68.6): widening turns it clean."""
    faint = sf.with_contact(sf.quiet(seed=70), 44.0)
    base = float(NOISE_FLOOR)
    assert classifier.classify_pass(faint, base, 30.0).contact is True
    assert classifier.classify_pass(faint, base * 1.5, 30.0).contact is False

    # With widening: pass 2 reads clean, so the probe finds no contact.
    plugin, toolhead, chip = _temp_setup(drag_setup, fake_printer, sensor_temp=65.0)
    with pytest.raises(fake_klippy.FakeCommandError, match="no contact within"):
        run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_result is None


def test_no_widening_within_band_is_prior_behavior(drag_setup, run_cmd, fake_printer):
    """Regression: within the +-15 degC band the faint contact is still
    detected exactly as before, with no widening warning."""
    plugin, toolhead, chip = _temp_setup(drag_setup, fake_printer, sensor_temp=30.0)
    run_cmd("PLR_DRAG_PROBE")  # deviation 5 -> no widening
    assert plugin.last_drag_result is not None
    assert plugin.last_drag_result["passes"] == 2
    responses = fake_printer.lookup_object("gcode").responses
    warns = [r for r in responses if "widening" in r]
    assert warns == []


def test_no_sensor_is_prior_behavior(drag_setup, run_cmd):
    """Regression: no configured sensor -> no widening, faint contact
    detected exactly as before (identical to the pre-covariate probe)."""
    plugin, toolhead, chip = drag_setup(
        chip_script=[quiet_stream, sf.with_contact(sf.quiet(seed=70), 44.0)],
        chip_default=quiet_stream,
    )
    run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_result is not None
    assert plugin.last_drag_result["passes"] == 2


def test_staged_temp_absent_skips_widening(drag_setup, run_cmd, fake_printer):
    """Regression: sensor present but no staged noise_floor_temp -> no
    widening (nothing to compare against)."""
    plugin, toolhead, chip = _temp_setup(
        drag_setup, fake_printer, sensor_temp=90.0, staged_temp=None
    )
    run_cmd("PLR_DRAG_PROBE")
    assert plugin.last_drag_result is not None
    assert plugin.last_drag_result["passes"] == 2
