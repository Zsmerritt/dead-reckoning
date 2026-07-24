"""Drag-oracle sensitivity calibration: the ``PLR_DRAG_CALIBRATE`` command.

Finds the MINIMUM sensitivity-knob value (= the most sensitive workable
threshold) that produces zero false contacts, entirely at a Z where
contact with any part is impossible.  Nothing here ever descends: every
pass runs at ``CLEAR_Z`` exactly, so an over-sensitive candidate fails
SAFELY — as a false contact in clear air, never as a crash into a part.

Sweep design (a sensitive-first port of Cartographer's touch calibration
sweep, macros/touch/calibrate.py):

* Start at the MOST sensitive knob (100) and sweep DOWNWARD in
  sensitivity only as far as needed.  Cartographer sweeps its threshold
  from a permissive start toward stricter values and accepts the first
  survivor (calibrate.py:335-396 ``_find_threshold``); our knob is
  inverted relative to the threshold multiplier (high knob = low
  multiplier = most sensitive), so "sweep toward stricter" means "step
  the knob DOWN".
* Two-tier screen-then-verify (calibrate.py:356-386): at each candidate,
  a quick SCREEN of ``screen_passes`` passes, and only for a screening
  survivor the longer VERIFY of ``verify_passes`` passes.
* Adaptive step (calibrate.py:85-95 ``calculate_step``): 20% of the
  current knob when failing badly (>= 2 false triggers), 10% when close
  (exactly 1), clamped to [2, 15] knob units.
* Verification early-exit on the first false contact (calibrate.py:205-215).
* Accept the FIRST (highest-sensitivity) surviving candidate; recommend
  staging ``drag_sensitivity = accepted - margin`` for a safety cushion.
* Exhaustion (the knob would go below 0) is NOT an exception
  (calibrate.py:398-412): an info log plus a copy-pasteable retry with a
  hint to re-run PLR_NOISE_TEST or check the mounting.

Every pass runs through :func:`plr.drag_probe.run_capture_pass` — the
same helper PLR_DRAG_PROBE uses — so the pass geometry, settle, and
capture lifecycle are identical to a real drag pass by construction.

START=1 consent is REQUIRED (it moves the toolhead, same policy as
PLR_NOISE_TEST): without it the command prints the plan and moves
nothing.
"""

import collections
import math

from . import classifier, drag_probe, tunables
from .touch_sequence import format_command, format_distance

# Sweep endpoints and the two-tier pass counts.
KNOB_START = 100.0
KNOB_FLOOR = 0.0
DEFAULT_SCREEN_PASSES = 3
DEFAULT_VERIFY_PASSES = 6
MIN_PASSES = 1
MAX_PASSES = 50

# Safety-margin default (knob units) subtracted from the accepted knob
# for the staged recommendation; floored at KNOB_FLOOR.
DEFAULT_MARGIN = 5.0
MAX_MARGIN = 50.0

# Adaptive knob step (carto calibrate.py:85-95 calculate_step): a
# proportion of the current knob, clamped to a knob-unit band.
KNOB_STEP_MIN = 2
KNOB_STEP_MAX = 15
KNOB_STEP_FRAC_BADLY = 0.20  # >= 2 false triggers while screening
KNOB_STEP_FRAC_CLOSE = 0.10  # exactly 1 false trigger

# CLEAR_Z must sit at least this far above the kinematic Z floor: the
# "guaranteed clear" standoff the whole command relies on.
CLEAR_Z_MIN_ABOVE_FLOOR = 5.0

# Everything one candidate evaluation needs; passed to the screen/verify
# helpers so the sweep reads as pure control flow.
CalibrateCtx = collections.namedtuple(
    "CalibrateCtx",
    [
        "plugin",
        "gcmd",
        "toolhead",
        "chip",
        "center_x",
        "center_y",
        "pass_length",
        "speed",
        "noise_floor",
        "screen_passes",
        "verify_passes",
    ],
)


def knob_step(knob, false_triggers):
    """Adaptive downward knob step (carto calibrate.py:85-95).

    ``KNOB_STEP_FRAC_BADLY`` (20%) of the current knob when failing badly
    (>= 2 false triggers in one screen), ``KNOB_STEP_FRAC_CLOSE`` (10%)
    when close (exactly 1), clamped to [KNOB_STEP_MIN, KNOB_STEP_MAX]
    knob units.  Integer-valued like carto's ``int(threshold * frac)`` so
    the swept knob sequence stays clean.
    """
    frac = KNOB_STEP_FRAC_BADLY if false_triggers >= 2 else KNOB_STEP_FRAC_CLOSE
    return min(KNOB_STEP_MAX, max(KNOB_STEP_MIN, int(knob * frac)))


def _classify_clear_pass(ctx, knob):
    """One lateral pass at CLEAR_Z through the production classifier.

    Returns a :class:`classifier.PassVerdict`.  A no-data capture or a
    degenerate/uncoverable sample stream cannot be calibrated around, so
    it aborts the whole command (the classifier's typed reason surfaces
    to the console) rather than being silently treated as clean.
    """
    capture = drag_probe.run_capture_pass(
        ctx.toolhead,
        ctx.chip,
        ctx.center_x,
        ctx.center_y,
        ctx.pass_length,
        ctx.speed,
    )
    if capture is None:
        _fail(
            ctx.plugin,
            ctx.gcmd,
            "PLR_DRAG_CALIBRATE aborted: accelerometer measured no data "
            "during a pass — check the chip wiring/config",
        )
    result = classifier.classify_pass(capture.samples, ctx.noise_floor, knob)
    if isinstance(result, classifier.PassInvalid):
        _fail(
            ctx.plugin,
            ctx.gcmd,
            "PLR_DRAG_CALIBRATE aborted: uncalibratable pass (%s: %s) — "
            "re-run PLR_NOISE_TEST or check the chip" % (result.reason, result.detail),
        )
    return result


def _screen(ctx, knob):
    """Quick screen: run ``screen_passes`` passes; count false contacts.

    Returns ``(false_triggers, peak_rms)``.  In guaranteed-clear air any
    contact verdict is a false trigger; the count drives the adaptive
    step (badly vs close).  All passes run — the count, not merely its
    presence, is needed to size the step (carto calibrate.py:361-367).
    """
    false_triggers = 0
    peak = 0.0
    for _ in range(ctx.screen_passes):
        result = _classify_clear_pass(ctx, knob)
        peak = max(peak, result.peak_rms)
        if result.contact:
            false_triggers += 1
    return false_triggers, peak


def _verify(ctx, knob):
    """Extended verify: all ``verify_passes`` must be contact-free.

    Returns ``(survived, passes_run, peak_rms)``.  Early-exits on the
    first false contact (carto calibrate.py:205-215), so a candidate that
    triggers on verify pass k runs exactly k passes.
    """
    peak = 0.0
    for attempt in range(ctx.verify_passes):
        result = _classify_clear_pass(ctx, knob)
        peak = max(peak, result.peak_rms)
        if result.contact:
            return False, attempt + 1, peak
    return True, ctx.verify_passes, peak


def _sweep(ctx):
    """Sensitive-first sweep; returns ``(accepted_knob, tested, peak)``.

    ``accepted_knob`` is None on exhaustion.  ``tested`` is the strictly
    decreasing sequence of knobs actually evaluated (highest first);
    ``peak`` is the worst clean-air peak RMS seen at the accepted knob.
    """
    knob = KNOB_START
    tested = []
    while knob >= KNOB_FLOOR:
        tested.append(knob)
        false_triggers, screen_peak = _screen(ctx, knob)
        if false_triggers:
            knob -= knob_step(knob, false_triggers)
            continue
        survived, _passes, verify_peak = _verify(ctx, knob)
        if not survived:
            # Verification early-exits at the first false contact, so the
            # observed count is one; step as "close" (carto calibrate.py
            # steps on any verify failure).
            knob -= knob_step(knob, 1)
            continue
        return knob, tested, max(screen_peak, verify_peak)
    return None, tested, 0.0


def _fail(plugin, gcmd, message):
    """Abort: null the calibrate result and raise a console error."""
    plugin.last_drag_calibrate = None
    raise gcmd.error(message)


def cmd_PLR_DRAG_CALIBRATE(plugin, gcmd):
    """PLR_DRAG_CALIBRATE [CHIP=] [SPEED=] [CLEAR_Z=] [SCREEN_PASSES=]
    [VERIFY_PASSES=] [MARGIN=] [PASS_LENGTH=] START=1 — find the most
    sensitive drag knob that never false-triggers at a clear Z."""
    try:
        chip_name = gcmd.get("CHIP", plugin.accel_chip_name)
        speed = gcmd.get_float(
            "SPEED", plugin.tunables["drag_speed"], above=0.0, maxval=100.0
        )
        screen_passes = gcmd.get_int(
            "SCREEN_PASSES", DEFAULT_SCREEN_PASSES, minval=MIN_PASSES, maxval=MAX_PASSES
        )
        verify_passes = gcmd.get_int(
            "VERIFY_PASSES", DEFAULT_VERIFY_PASSES, minval=MIN_PASSES, maxval=MAX_PASSES
        )
        margin = gcmd.get_float("MARGIN", DEFAULT_MARGIN, minval=0.0, maxval=MAX_MARGIN)
        pass_length = gcmd.get_float(
            "PASS_LENGTH",
            drag_probe.DEFAULT_PASS_LENGTH,
            minval=drag_probe.MIN_PASS_LENGTH,
            maxval=drag_probe.MAX_PASS_LENGTH,
        )
        toolhead = drag_probe.check_motion_gates(plugin, gcmd, "PLR_DRAG_CALIBRATE")
        chip = drag_probe.resolve_accel_chip(plugin, gcmd, chip_name)
        noise_floor = plugin.noise_floor_rms
        if noise_floor is None:
            raise gcmd.error(
                "no accelerometer noise floor on record — run PLR_NOISE_TEST first"
            )
        if plugin.z_position_min is None or not math.isfinite(plugin.z_position_min):
            raise gcmd.error(
                "no finite Z floor configured: set position_min in "
                "[stepper_z] (or minimum_z_position in [printer])"
            )
    except gcmd.error:
        plugin.last_drag_calibrate = None
        raise

    start_pos = toolhead.get_position()
    current_z = start_pos[2]
    clear_z = gcmd.get_float("CLEAR_Z", current_z)
    floor_limit = plugin.z_position_min + CLEAR_Z_MIN_ABOVE_FLOOR
    if clear_z < floor_limit:
        _fail(
            plugin,
            gcmd,
            "PLR_DRAG_CALIBRATE refused: CLEAR_Z %.3f is below the required "
            "clear standoff %.3f (kinematic floor %.3f + %.1f mm) — this "
            "command only runs where contact is impossible"
            % (clear_z, floor_limit, plugin.z_position_min, CLEAR_Z_MIN_ABOVE_FLOOR),
        )
    if clear_z < current_z:
        _fail(
            plugin,
            gcmd,
            "PLR_DRAG_CALIBRATE refused: CLEAR_Z %.3f is below the current Z "
            "%.3f — this command never descends; move up first or raise "
            "CLEAR_Z" % (clear_z, current_z),
        )

    if not gcmd.get_int("START", 0):
        gcmd.respond_info(
            "PLR_DRAG_CALIBRATE plan (no motion yet):\n"
            "  raise Z to CLEAR_Z=%.3f (upward only), then sweep the "
            "sensitivity knob DOWN from %.0f, running %d screen + up to %d "
            "verify lateral passes of %.1f mm at %.1f mm/s at CLEAR_Z until "
            "one survives with zero false contacts.\n"
            "  accepts the highest surviving knob; stages "
            "drag_sensitivity = accepted - MARGIN=%.1f for SAVE_CONFIG.\n"
            "  every pass runs at CLEAR_Z exactly — this command never "
            "descends.\n"
            "Re-run with START=1 to consent to this motion."
            % (
                clear_z,
                KNOB_START,
                screen_passes,
                verify_passes,
                pass_length,
                speed,
                margin,
            )
        )
        return

    # Raise to the clear standoff (upward only; never a descent).
    if clear_z > current_z:
        toolhead.manual_move([None, None, clear_z], drag_probe.LIFT_SPEED)
        toolhead.wait_moves()

    gcmd.respond_info(
        "PLR_DRAG_CALIBRATE: sweeping the sensitivity knob down from %.0f at "
        "CLEAR_Z %.3f (%d screen + up to %d verify passes of %.1f mm at %.1f "
        "mm/s, noise floor %.3f mm/s^2)"
        % (
            KNOB_START,
            clear_z,
            screen_passes,
            verify_passes,
            pass_length,
            speed,
            noise_floor,
        )
    )

    ctx = CalibrateCtx(
        plugin=plugin,
        gcmd=gcmd,
        toolhead=toolhead,
        chip=chip,
        center_x=start_pos[0],
        center_y=start_pos[1],
        pass_length=pass_length,
        speed=speed,
        noise_floor=noise_floor,
        screen_passes=screen_passes,
        verify_passes=verify_passes,
    )
    accepted, tested, peak = _sweep(ctx)

    if accepted is None:
        # Exhaustion is not an error (carto calibrate.py:398-412): even
        # the least sensitive knob false-triggered in clear air, which
        # points at the noise floor or the mounting, not the sweep.
        retry = format_command(
            "PLR_DRAG_CALIBRATE",
            [
                ("START", "1"),
                ("CHIP", chip_name),
                ("SPEED", "%.2f" % (speed,)),
                ("CLEAR_Z", "%.3f" % (clear_z,)),
            ],
        )
        plugin.last_drag_calibrate = None
        gcmd.respond_info(
            "PLR_DRAG_CALIBRATE: no knob down to %.0f survived — every "
            "candidate false-triggered in clear air (knobs tried: %s).\n"
            "The noise floor may be stale or the accel mount loose: re-run "
            "PLR_NOISE_TEST, or check the chip mounting, then retry:\n  %s"
            % (KNOB_FLOOR, ", ".join("%.0f" % (k,) for k in tested), retry)
        )
        return

    recommended = max(KNOB_FLOOR, accepted - margin)
    threshold = noise_floor * classifier.multiplier(accepted)
    headroom = threshold / peak if peak > 0.0 else float("inf")

    configfile = plugin.printer.lookup_object("configfile")
    configfile.set("plr", "drag_sensitivity", tunables.format_value(recommended))
    plugin.note_pending_save("drag_sensitivity")
    plugin.tunables["drag_sensitivity"] = recommended
    plugin.last_drag_calibrate = {
        "accepted_knob": accepted,
        "recommended": recommended,
        "clear_z": clear_z,
        "tested": list(tested),
    }

    gcmd.respond_info(
        "PLR_DRAG_CALIBRATE: accepted knob %.0f (highest with zero false "
        "contacts; knobs tried: %s)\n"
        "  threshold at knob %.0f = %.3f mm/s^2 (noise floor x %.2f); worst "
        "clear-air peak %.3f leaves %sx headroom\n"
        "  staged drag_sensitivity = %.0f (accepted - MARGIN %.1f, floor %.0f)\n"
        "The SAVE_CONFIG command will update the printer config file with "
        "the above and restart the printer."
        % (
            accepted,
            ", ".join("%.0f" % (k,) for k in tested),
            accepted,
            threshold,
            classifier.multiplier(accepted),
            peak,
            format_distance(headroom),
            recommended,
            margin,
            KNOB_FLOOR,
        )
    )
