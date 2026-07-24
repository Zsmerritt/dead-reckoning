"""Drag-probe surface location: the ``PLR_DRAG_PROBE`` command.

Locates the top of a solidified part with an accelerometer instead of a
descending probe: repeated bounded lateral passes at a fixed Z, stepping
the staircase down by ``Z_STEP`` between passes, until the pass sample
window classifies as contact (:mod:`plr.classifier`).

Safety model (fixed; the recovery executor's true-Z arithmetic depends
on it):

* Accelerometer data arrives in BATCHES (klippy's internal-client API,
  klippy/extras/adxl345.py:251-254 ``start_internal_client``; samples
  land via bulk batch messages), so there is NO real-time halt.  Every
  lateral pass therefore runs at a FIXED Z — a pass physically cannot
  descend — and classification happens BETWEEN passes on the complete
  window, with no latency pressure.
* Clean pass -> descend exactly Z_STEP (one ``manual_move``,
  klippy/toolhead.py:410-416) and repeat.
* Contact -> STOP.  No further descent; lift by a fixed 2 x Z_STEP
  clearance bounded above by the starting height (the start Z is the
  presumed-safe standoff, so the lift never leaves the tested
  envelope).  ``trigger_z`` reported is the Z of the LAST CLEAN pass:
  the surface lies within ``(trigger_z - Z_STEP, trigger_z]``, and the
  conservative endpoint (``trigger_z``, the shallowest the surface can
  be) is exactly what the executor's overshoot-tolerant arithmetic
  expects (overshoot <= Z_STEP by construction).
* HARD ITERATION BOUND computed UP FRONT: ceil(available_travel /
  Z_STEP) where available travel is current Z minus the Z floor.  The
  floor is the kinematic limit ([stepper_z] position_min /
  [printer] minimum_z_position, cached at config time by
  plr.setup_checks.z_position_min) plus a 1 x Z_STEP reserve.  No
  descent is ever commanded below the floor — checked before EVERY
  descent; klippy's own kinematic limit check
  (klippy/kinematics/cartesian.py:97-105 ``_check_endstops`` raising
  "Move out of range") is the backstop, never the mechanism.
* An invalid pass (degenerate samples — see
  classifier.validate_pass_samples) ABORTS the probe.  "Assume clean
  and descend" is the unsafe direction and is never taken.

Consent parity: like klippy's own PROBE, this command is a primitive —
it runs when typed, with no START= consent parameter.  It moves a few
millimetres laterally and only ever descends in Z_STEP increments
inside the computed envelope; the scripted multi-move diagnostics
(PLR_PROBE_TEST, PLR_NOISE_TEST) keep their START=1 gates.

Frozen invocation contract (the plrd recovery plan emits exactly this):
``PLR_DRAG_PROBE CHIP=<name> SPEED=<mm/s> Z_STEP=<mm>
SENSITIVITY=<0-100>`` — every argument optional, falling back to the
``[plr]`` tunables.  ``CHIP`` accepts both bare (``adxl345``) and
quoted spaced (``CHIP="adxl345 bed"``) section names: PLR_DRAG_PROBE is
an extended command, so klippy re-parses its parameters with
shell-style quoting (klippy/gcode.py:145-151 wraps non-traditional
commands; klippy/gcode.py:266-281 ``_get_extended_params`` uses
``shlex(..., posix=True)`` with ``whitespace_split``), which strips the
quotes and preserves the embedded space and the value's case.

Success/failure surface (read by plrd through the API socket exactly
like probe status): ``get_status()['last_drag_result']`` is
``{"trigger_z": float, "passes": int, "confidence": float}`` after a
successful probe (toolhead-frame Z, mirroring how probe.py records halt
positions in toolhead coordinates rather than the gcode_move frame);
on any failure or refusal it is None and ``last_drag_error`` carries a
console-identical string.  Failures also raise ``gcmd.error`` so a
caller script can catch them, mirroring klippy's "No trigger on probe"
convention.
"""

import math

from . import classifier
from .probe_test import _print_active

# Pass geometry: total back-and-forth path length in mm, centered on
# the current XY (out +L/4, across to -L/4, back to center = L total).
DEFAULT_PASS_LENGTH = 8.0
MIN_PASS_LENGTH = 2.0
MAX_PASS_LENGTH = 20.0

# Post-contact clearance lift, in Z_STEP units (bounded by the start Z).
CLEARANCE_STEPS = 2.0

# Z-floor reserve above the kinematic limit, in Z_STEP units.
FLOOR_RESERVE_STEPS = 1.0

# Settle time between arriving at a pass Z and starting the capture, so
# descent transients do not pollute the pass window (resonance_tester
# settles the same way before capturing, resonance_tester.py:328-329).
SETTLE_SECONDS = 0.25

# Lift/restore speed (mm/s) for upward-only moves back into already
# proven-clear heights; deliberately fixed, not a tunable.
LIFT_SPEED = 5.0


def travel_seconds(distance_mm, speed_mm_s):
    """Duration in seconds of a straight move; used for command timeouts.

    Raises ValueError on a non-positive speed or negative distance — both
    indicate a malformed g-code parameter, not a physics question.
    """
    if speed_mm_s <= 0:
        raise ValueError("SPEED=%s must be positive" % (speed_mm_s,))
    if distance_mm < 0:
        raise ValueError("distance %s must not be negative" % (distance_mm,))
    return distance_mm / speed_mm_s


def iteration_bound(start_z, floor_z, z_step):
    """Hard cap on drag passes: ceil(available_travel / z_step).

    Computed up front from the starting height and the Z floor so the
    staircase loop is bounded even if every classification is wrong.
    Raises ValueError when there is no travel at all (start at/below
    floor) or on a non-positive step — refusal, not a zero bound.
    """
    if z_step <= 0.0:
        raise ValueError("z_step %s must be positive" % (z_step,))
    if start_z <= floor_z:
        raise ValueError(
            "start Z %.3f is at/below the Z floor %.3f — no probing envelope"
            % (start_z, floor_z)
        )
    return int(math.ceil((start_z - floor_z) / z_step))


def resolve_accel_chip(plugin, gcmd, chip_name):
    """Resolve ``chip_name`` to a live accelerometer printer object.

    Refusals are console errors listing the chip sections found at
    config time (setup_checks.list_accel_chips), so a typo'd CHIP= is
    a one-glance fix.  Mirrors resonance_tester's duck-type check that
    the object really is an accelerometer
    (klippy/extras/resonance_tester.py:304-307).
    """
    found = "unknown"
    for res in plugin.static_check_results:
        if res.name == "accel chips":
            found = res.detail
    if not chip_name:
        raise gcmd.error(
            "no accel chip: set accel_chip in [plr] or pass CHIP= "
            "(chip sections found: %s)" % (found,)
        )
    chip = plugin.printer.lookup_object(chip_name, None)
    if chip is None:
        raise gcmd.error(
            "accel chip '%s' not found (chip sections found: %s)" % (chip_name, found)
        )
    if not hasattr(chip, "start_internal_client"):
        raise gcmd.error("'%s' is not an accelerometer" % (chip_name,))
    return chip


def check_motion_gates(plugin, gcmd, command):
    """Shared invocation gates: refuse while printing / not homed.

    Same signals as PLR_PROBE_TEST: print_stats state with idle_timeout
    fallback (probe_test._print_active), and toolhead homed_axes
    (klippy/toolhead.py:503-513 get_status merged with the kinematics
    status, the signal probe.py:352-355 checks for z).
    """
    printer = plugin.printer
    if _print_active(printer):
        raise gcmd.error(
            "%s refused: a print is active (it moves the toolhead); "
            "wait for the print to finish or cancel it" % (command,)
        )
    toolhead = printer.lookup_object("toolhead")
    eventtime = printer.get_reactor().monotonic()
    homed = toolhead.get_status(eventtime)["homed_axes"]
    if any(axis not in homed for axis in "xyz"):
        raise gcmd.error(
            "%s refused: printer must be homed (homed axes: '%s', need "
            "xyz) — run G28 first" % (command, homed)
        )
    return toolhead


def run_capture_pass(toolhead, chip, center_x, center_y, pass_length, speed):
    """One bounded lateral pass at the CURRENT Z, capturing throughout.

    Canonical internal-client shape (resonance_tester.py:328-352):
    settle, start client, run the moves, finish_measurements (which
    itself waits for motion, adxl345.py:42-46), then read the batch.
    Returns the sample list, or None when the chip produced no usable
    batches (adxl345.py:55-71 has_valid_samples).  The segment is
    center -> +L/4 -> -L/4 -> center: symmetric about the invocation
    XY and exactly ``pass_length`` mm of total travel.
    """
    seg = pass_length / 4.0
    toolhead.wait_moves()
    toolhead.dwell(SETTLE_SECONDS)
    aclient = chip.start_internal_client()
    try:
        toolhead.manual_move([center_x + seg, center_y, None], speed)
        toolhead.manual_move([center_x - seg, center_y, None], speed)
        toolhead.manual_move([center_x, center_y, None], speed)
    finally:
        aclient.finish_measurements()
    if not aclient.has_valid_samples():
        return None
    return aclient.get_samples()


def _fail(plugin, gcmd, message):
    """Record a failure in status and raise it as a console error.

    Every refusal/abort goes through here so plrd sees a null
    last_drag_result plus last_drag_error even for pre-motion gate
    refusals — a stale success must never survive a failed invocation.
    """
    plugin.last_drag_result = None
    plugin.last_drag_error = message
    raise gcmd.error(message)


def cmd_PLR_DRAG_PROBE(plugin, gcmd):
    """PLR_DRAG_PROBE [CHIP=] [SPEED=] [Z_STEP=] [SENSITIVITY=]
    [PASS_LENGTH=] — locate the part surface with the drag oracle."""
    toolhead = None
    try:
        # Argument ranges mirror the [plr] tunable schema exactly
        # (tunables.TUNABLES); absent args fall back to the live
        # tunable values, per the frozen plrd invocation contract.
        chip_name = gcmd.get("CHIP", plugin.accel_chip_name)
        speed = gcmd.get_float(
            "SPEED", plugin.tunables["drag_speed"], above=0.0, maxval=100.0
        )
        z_step = gcmd.get_float(
            "Z_STEP", plugin.tunables["drag_z_step"], above=0.0, maxval=0.2
        )
        sensitivity = gcmd.get_float(
            "SENSITIVITY",
            plugin.tunables["drag_sensitivity"],
            minval=0.0,
            maxval=100.0,
        )
        pass_length = gcmd.get_float(
            "PASS_LENGTH",
            DEFAULT_PASS_LENGTH,
            minval=MIN_PASS_LENGTH,
            maxval=MAX_PASS_LENGTH,
        )
        toolhead = check_motion_gates(plugin, gcmd, "PLR_DRAG_PROBE")
        chip = resolve_accel_chip(plugin, gcmd, chip_name)
        noise_floor = plugin.noise_floor_rms
        if noise_floor is None:
            raise gcmd.error(
                "no accelerometer noise floor on record — run "
                "PLR_NOISE_TEST (then SAVE_CONFIG) first"
            )
        if plugin.z_position_min is None or not math.isfinite(plugin.z_position_min):
            raise gcmd.error(
                "no finite Z floor configured: set position_min in "
                "[stepper_z] (or minimum_z_position in [printer])"
            )
    except gcmd.error as e:
        # Gate refusals must surface in status too, not just console.
        plugin.last_drag_result = None
        plugin.last_drag_error = str(e)
        raise

    start_pos = toolhead.get_position()  # toolhead frame, as probe.py
    start_z = start_pos[2]
    floor_z = plugin.z_position_min + FLOOR_RESERVE_STEPS * z_step
    if start_z <= floor_z:
        _fail(
            plugin,
            gcmd,
            "PLR_DRAG_PROBE refused: start Z %.3f is at/below the Z floor "
            "%.3f (kinematic limit %.3f + %.0fx Z_STEP reserve) — move up "
            "first" % (start_z, floor_z, plugin.z_position_min, FLOOR_RESERVE_STEPS),
        )
    bound = iteration_bound(start_z, floor_z, z_step)
    descend_speed = plugin.tunables["probe_speed"]
    threshold = noise_floor * classifier.multiplier(sensitivity)
    gcmd.respond_info(
        "PLR_DRAG_PROBE: staircase from Z %.3f, floor %.3f, step %.3f "
        "(max %d passes), pass %.1f mm at %.1f mm/s, threshold %.2f mm/s^2 "
        "(noise floor %.2f x %.2f at sensitivity %.0f)"
        % (
            start_z,
            floor_z,
            z_step,
            bound,
            pass_length,
            speed,
            threshold,
            noise_floor,
            classifier.multiplier(sensitivity),
            sensitivity,
        )
    )

    z = start_z
    last_clean_z = None
    contact = None
    passes_run = 0
    for pass_index in range(bound):
        samples = run_capture_pass(
            toolhead, chip, start_pos[0], start_pos[1], pass_length, speed
        )
        passes_run += 1
        if samples is None:
            _restore_z(toolhead, start_z)
            _fail(
                plugin,
                gcmd,
                "PLR_DRAG_PROBE aborted at Z %.3f: accelerometer measured "
                "no data (pass %d) — Z restored to %.3f" % (z, passes_run, start_z),
            )
        result = classifier.classify_pass(samples, noise_floor, sensitivity)
        if isinstance(result, classifier.PassInvalid):
            # Never "assume clean and descend": a pass that cannot be
            # classified ends the probe with the toolhead back at the
            # proven-safe starting height.
            _restore_z(toolhead, start_z)
            _fail(
                plugin,
                gcmd,
                "PLR_DRAG_PROBE aborted at Z %.3f: invalid pass (%s: %s) "
                "— Z restored to %.3f" % (z, result.reason, result.detail, start_z),
            )
        if result.contact:
            if last_clean_z is None:
                # Contact on the very first pass: there is no clean Z
                # to report, so the surface bracket the executor needs
                # does not exist.  The toolhead is still at start_z.
                _fail(
                    plugin,
                    gcmd,
                    "PLR_DRAG_PROBE failed: contact on the first pass at "
                    "the starting height %.3f — start higher, or lower "
                    "SENSITIVITY if this is noise" % (start_z,),
                )
            contact = result
            break
        last_clean_z = z
        if pass_index == bound - 1:
            break  # iteration bound reached; never descend without a
            # following pass to classify at the new height
        next_z = z - z_step
        # Checked before EVERY descent; klippy's kinematic limit is the
        # backstop, never the mechanism.
        if next_z < floor_z:
            break
        toolhead.manual_move([None, None, next_z], descend_speed)
        z = next_z

    if contact is None:
        _restore_z(toolhead, start_z)
        _fail(
            plugin,
            gcmd,
            "PLR_DRAG_PROBE: no contact within envelope after %d passes "
            "(Z %.3f down to %.3f) — check SENSITIVITY / part location; "
            "Z restored to %.3f" % (passes_run, start_z, z, start_z),
        )

    clearance_z = min(start_z, z + CLEARANCE_STEPS * z_step)
    if clearance_z > z:
        toolhead.manual_move([None, None, clearance_z], LIFT_SPEED)
    toolhead.wait_moves()
    plugin.last_drag_result = {
        "trigger_z": last_clean_z,
        "passes": passes_run,
        "confidence": contact.confidence,
    }
    plugin.last_drag_error = None
    gcmd.respond_info(
        "PLR_DRAG_PROBE: contact at Z %.3f (pass %d, ratio %.2f, "
        "confidence %.2f)\n"
        "trigger_z = %.3f (last clean pass; surface within (%.3f, %.3f])\n"
        "Z lifted to %.3f"
        % (
            z,
            passes_run,
            contact.ratio,
            contact.confidence,
            last_clean_z,
            last_clean_z - z_step,
            last_clean_z,
            clearance_z,
        )
    )


def _restore_z(toolhead, start_z):
    """Lift straight back to the starting height (upward-only, safe)."""
    current = toolhead.get_position()
    if current[2] < start_z:
        toolhead.manual_move([None, None, start_z], LIFT_SPEED)
    toolhead.wait_moves()
