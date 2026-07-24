"""Accelerometer noise-floor measurement: the ``PLR_NOISE_TEST`` command.

Measures the baseline the drag oracle thresholds against and stages it
for SAVE_CONFIG.  Two captures, both through klippy's internal-client
API (klippy/extras/adxl345.py:251-254 ``start_internal_client``;
resonance_tester.py:328-352 is the canonical start -> move -> finish ->
analyze shape):

a) STILL: ~2 s with the toolhead parked (motors energized, no motion);
b) MOVING: N no-contact lateral passes at the drag SPEED at the CURRENT
   Z — the same pass geometry as PLR_DRAG_PROBE, capturing throughout.

The value persisted as ``noise_floor_rms`` is the MOVING capture's RMS,
not the still one: drag passes classify samples taken while the
toolhead is moving, so the baseline must include motion-correlated
vibration (stepper harmonics, frame resonance, belt noise) or every
pass would false-trigger on the machine's own motion.  The still RMS is
persisted alongside for diagnostics only.

Persisted keys (configfile.set under section "plr"; klippy's SAVE_CONFIG
autosave mechanism, klippy/configfile.py:311-324):

* ``noise_floor_rms`` — moving-capture whole-stream RMS (mm/s^2) of the
  classifier-preprocessed magnitude.  REQUIRED by the drag oracle and
  the value plrd's validation reads.
* ``noise_floor_still_rms`` — still-capture RMS; a large moving/still
  gap is a mechanical-health hint (loose accel mount, resonant frame).
* ``noise_floor_peak`` — moving-capture max windowed RMS: the exact
  statistic classify_pass compares against its threshold, so
  peak/threshold headroom at the chosen sensitivity is readable
  directly from the report.
* ``noise_floor_speed`` — the SPEED (mm/s) the moving baseline was
  captured at.  Consumed by plrd's plan validation to warn when a
  recovery probe would drag at a different speed than the floor was
  measured at (the baseline is speed-dependent).

Degenerate captures (too few samples, non-finite, constant, collapsed
sample rate — classifier.validate_pass_samples' taxonomy) refuse the
whole measurement; nothing is staged.

START=1 consent is REQUIRED (unlike PLR_DRAG_PROBE): this command moves
the toolhead through a scripted multi-pass sequence, same policy as
PLR_PROBE_TEST.  Without START=1 it prints the plan and moves nothing.

WARNING (also printed in the plan and report): run this with the
toolhead well away from any printed part.  The command cannot know
where parts are — a pass that clips one poisons the baseline high and
the drag oracle would then under-trigger.
"""

from . import classifier, drag_probe

# Still-capture duration bounds (seconds).
DEFAULT_DURATION = 2.0
MIN_DURATION = 0.5
MAX_DURATION = 10.0

# Number of no-contact lateral passes in the moving capture: enough
# repetitions to average pass-to-pass variation without turning the
# test into a print job.
MOVE_PASSES = 4

CLEARANCE_WARNING = (
    "WARNING: run PLR_NOISE_TEST with the toolhead well away from any "
    "printed part — this command cannot know where parts are, and a "
    "pass that touches one corrupts the noise floor"
)


def cmd_PLR_NOISE_TEST(plugin, gcmd):
    """PLR_NOISE_TEST [CHIP=] [SPEED=] [DURATION=] START=1 — measure
    the accel noise floor and stage noise_floor_* for SAVE_CONFIG."""
    chip_name = gcmd.get("CHIP", plugin.accel_chip_name)
    speed = gcmd.get_float(
        "SPEED", plugin.tunables["drag_speed"], above=0.0, maxval=100.0
    )
    duration = gcmd.get_float(
        "DURATION", DEFAULT_DURATION, minval=MIN_DURATION, maxval=MAX_DURATION
    )
    toolhead = drag_probe.check_motion_gates(plugin, gcmd, "PLR_NOISE_TEST")
    chip = drag_probe.resolve_accel_chip(plugin, gcmd, chip_name)
    pass_length = drag_probe.DEFAULT_PASS_LENGTH
    if not gcmd.get_int("START", 0):
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PLR_NOISE_TEST plan (no motion yet):\n"
            "  a) still capture for %.1f s at X=%.3f Y=%.3f Z=%.3f\n"
            "  b) %d no-contact passes of %.1f mm at %.1f mm/s at the "
            "current Z (same geometry as PLR_DRAG_PROBE)\n"
            "  then stage noise_floor_rms / noise_floor_still_rms / "
            "noise_floor_peak / noise_floor_speed for SAVE_CONFIG.\n"
            "%s\n"
            "Re-run with START=1 to consent to this motion."
            % (
                duration,
                pos[0],
                pos[1],
                pos[2],
                MOVE_PASSES,
                pass_length,
                speed,
                CLEARANCE_WARNING,
            )
        )
        return
    gcmd.respond_info(CLEARANCE_WARNING)

    # a) Still capture: park, then let the chip stream for DURATION.
    toolhead.wait_moves()
    aclient = chip.start_internal_client()
    try:
        toolhead.dwell(duration)
    finally:
        aclient.finish_measurements()
    still = _capture_stats(gcmd, aclient, "still")

    # b) Moving capture: the drag-pass geometry, MOVE_PASSES times.
    pos = toolhead.get_position()
    aclient = chip.start_internal_client()
    try:
        seg = pass_length / 4.0
        for _ in range(MOVE_PASSES):
            toolhead.manual_move([pos[0] + seg, pos[1], None], speed)
            toolhead.manual_move([pos[0] - seg, pos[1], None], speed)
            toolhead.manual_move([pos[0], pos[1], None], speed)
    finally:
        aclient.finish_measurements()
    moving = _capture_stats(gcmd, aclient, "moving")

    # Stage the keys, then mirror them live on the plugin so
    # PLR_DRAG_PROBE works this session without waiting for the
    # SAVE_CONFIG restart (same live-then-persist convention as
    # PLR_PROBE_TEST's probe_resolution).  noise_floor_speed records
    # the SPEED the moving baseline was captured at: plrd's plan
    # validation warns when a recovery probe would run at a different
    # drag speed than the floor was measured at.
    configfile = plugin.printer.lookup_object("configfile")
    staged = [
        ("noise_floor_rms", moving.rms),
        ("noise_floor_still_rms", still.rms),
        ("noise_floor_peak", moving.peak_rms),
        ("noise_floor_speed", speed),
    ]
    # Temperature covariate: when a sensor is configured AND currently
    # readable, stage the temperature the baseline was captured at, so
    # PLR_DRAG_PROBE can widen the threshold if the machine has since
    # drifted.  No sensor configured, or no reading, stages nothing
    # (the covariate is skipped silently at probe time — no guessing).
    baseline_temp = drag_probe.read_temp(plugin, plugin.noise_floor_temp_sensor)
    if baseline_temp is not None:
        staged.append(("noise_floor_temp", baseline_temp))
    for option, value in staged:
        configfile.set("plr", option, "%.6f" % (value,))
        plugin.note_pending_save(option)
    plugin.noise_floor_rms = moving.rms
    plugin.noise_floor_still_rms = still.rms
    plugin.noise_floor_peak = moving.peak_rms
    plugin.noise_floor_speed = speed
    if baseline_temp is not None:
        plugin.noise_floor_temp = baseline_temp

    sensitivity = plugin.tunables["drag_sensitivity"]
    mult = classifier.multiplier(sensitivity)
    gcmd.respond_info(
        "plr noise test (chip %s):\n"
        "  noise_floor_rms       = %.3f mm/s^2 (moving baseline — the "
        "drag threshold reference)\n"
        "  noise_floor_still_rms = %.3f mm/s^2\n"
        "  noise_floor_peak      = %.3f mm/s^2 (max windowed RMS while "
        "moving)\n"
        "  noise_floor_speed     = %.1f mm/s (baseline capture speed; "
        "plans warn on drag-speed mismatch)\n"
        "  drag threshold at sensitivity %.0f: %.3f mm/s^2 (rms x %.2f); "
        "moving peak leaves %.1fx headroom\n"
        "%s\n"
        "The SAVE_CONFIG command will update the printer config file\n"
        "with the above and restart the printer."
        % (
            chip_name,
            moving.rms,
            still.rms,
            moving.peak_rms,
            speed,
            sensitivity,
            moving.rms * mult,
            mult,
            (moving.rms * mult) / moving.peak_rms,
            CLEARANCE_WARNING,
        )
    )


def _capture_stats(gcmd, aclient, label):
    """Validated StreamStats for one finished capture, or gcmd.error.

    Degenerate captures refuse the measurement with the classifier's
    typed reason; a refused noise test stages nothing.
    """
    if not aclient.has_valid_samples():
        raise gcmd.error(
            "PLR_NOISE_TEST: accelerometer measured no data during the "
            "%s capture — check the chip wiring/config" % (label,)
        )
    stats = classifier.stream_stats(aclient.get_samples())
    if isinstance(stats, classifier.PassInvalid):
        raise gcmd.error(
            "PLR_NOISE_TEST: degenerate %s capture (%s: %s) — nothing "
            "staged" % (label, stats.reason, stats.detail)
        )
    if stats.rms <= 0.0:
        raise gcmd.error(
            "PLR_NOISE_TEST: %s capture RMS is zero — chip not "
            "measuring? nothing staged" % (label,)
        )
    return stats
