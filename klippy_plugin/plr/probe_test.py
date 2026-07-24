"""Probe repeatability diagnostic: the PLR_PROBE_TEST command.

Runs N probe cycles at the current XY position and reports their
spread, then stages the measured ``probe_resolution`` into the [plr]
autosave section for SAVE_CONFIG.  The probing loop mirrors klippy's
own PROBE_ACCURACY implementation
(klippy/extras/probe.py:125-169 ``ProbeCommandHelper.cmd_PROBE_ACCURACY``):

* build a parameter dict from the live command with ``SAMPLES`` forced
  to ``'1'`` and wrap it in a dummy gcode command via
  ``gcode.create_gcode_command("", "", params)`` (probe.py:137-140);
* ``probe.start_probe_session(fo_gcmd)`` /
  ``session.run_probe(fo_gcmd)`` per sample, retracting between
  samples with ``toolhead.manual_move`` at the probe's lift_speed
  (probe.py:142-151);
* ``session.pull_probed_results()`` yields result objects whose
  ``bed_z`` is the trigger height, then ``session.end_probe_session()``
  (probe.py:152-153).

Safety gates, all client-side and all before any motion:

* refuses while a print is active (print_stats state, falling back to
  idle_timeout state when [virtual_sdcard] is absent);
* refuses unless x, y and z are homed (toolhead.get_status
  ``homed_axes``, the same signal probe.py:352-355 checks for z);
* refuses probe_method adxl_drag (no descending probe to test);
* does NOTHING without an explicit ``START=1`` — without it the
  command only prints what it would do (motion consent).
"""

MIN_SAMPLES = 3
MAX_SAMPLES = 50
DEFAULT_SAMPLES = 10

# Floor for the persisted probe_resolution (mm).  A perfectly repeating
# probe still cannot resolve below roughly a microstep; claiming better
# than 5 microns would let the planner trust noise.
MIN_PROBE_RESOLUTION = 0.005


def validate_sample_count(count, minimum=MIN_SAMPLES):
    """Validate the SAMPLES= g-code parameter; return it unchanged.

    Raises ValueError with a console-ready message when the count is too
    low to compute a meaningful spread.
    """
    if count < minimum:
        raise ValueError(
            "SAMPLES=%d is too low: at least %d probe samples are needed"
            % (count, minimum)
        )
    return count


def compute_stats(heights):
    """Range, population standard deviation, and median of trigger heights.

    Same statistics PROBE_ACCURACY prints (klippy/extras/probe.py:154-164
    computes range/average/median and a population sigma).
    """
    if not heights:
        raise ValueError("compute_stats: no probe samples")
    n = len(heights)
    mean = sum(heights) / n
    range_value = max(heights) - min(heights)
    sigma = (sum((h - mean) ** 2 for h in heights) / n) ** 0.5
    ordered = sorted(heights)
    mid = n // 2
    if n % 2:
        median = ordered[mid]
    else:
        median = (ordered[mid - 1] + ordered[mid]) / 2.0
    return {"range": range_value, "stddev": sigma, "median": median, "mean": mean}


def resolution_from_stddev(sigma, minimum=MIN_PROBE_RESOLUTION):
    """probe_resolution staged for SAVE_CONFIG: max(2*sigma, floor)."""
    return max(2.0 * sigma, minimum)


def _print_active(printer):
    """True when a print is running/paused; None-safe on bare configs.

    Prefers print_stats (klippy/extras/print_stats.py:118 exposes
    'state' in {standby, printing, paused, complete, error, cancelled};
    only exists with [virtual_sdcard]).  Falls back to idle_timeout
    (klippy/extras/idle_timeout.py:34-40: state 'Printing' means motion
    was commanded within the last idle window — coarser, so a recent
    manual move can also read as active; documented caveat).
    """
    print_stats = printer.lookup_object("print_stats", None)
    if print_stats is not None:
        eventtime = printer.get_reactor().monotonic()
        return print_stats.get_status(eventtime)["state"] in ("printing", "paused")
    idle_timeout = printer.lookup_object("idle_timeout", None)
    if idle_timeout is not None:
        eventtime = printer.get_reactor().monotonic()
        return idle_timeout.get_status(eventtime)["state"] == "Printing"
    return False


def cmd_PLR_PROBE_TEST(plugin, gcmd):
    """PLR_PROBE_TEST [SAMPLES=10] START=1 — probe repeatability test."""
    printer = plugin.printer
    if plugin.probe_method == "adxl_drag":
        raise gcmd.error(
            "PLR_PROBE_TEST needs a descending probe (probe_method tap or "
            "load_cell); probe_method is adxl_drag — the drag oracle has "
            "its own diagnostics (PLR_NOISE_TEST / PLR_DRAG_PROBE)"
        )
    samples = gcmd.get_int(
        "SAMPLES", DEFAULT_SAMPLES, minval=MIN_SAMPLES, maxval=MAX_SAMPLES
    )
    validate_sample_count(samples)
    if _print_active(printer):
        raise gcmd.error(
            "PLR_PROBE_TEST refused: a print is active (it moves the "
            "toolhead); wait for the print to finish or cancel it"
        )
    toolhead = printer.lookup_object("toolhead")
    eventtime = printer.get_reactor().monotonic()
    homed = toolhead.get_status(eventtime)["homed_axes"]
    if any(axis not in homed for axis in "xyz"):
        raise gcmd.error(
            "PLR_PROBE_TEST refused: printer must be homed (homed axes: "
            "'%s', need xyz) — run G28 first" % (homed,)
        )
    probe = printer.lookup_object("probe", None)
    if probe is None:
        raise gcmd.error(
            "no probe object — is the [%s] section present?"
            % ("probe" if plugin.probe_method == "tap" else "load_cell_probe")
        )
    probe_speed = plugin.tunables["probe_speed"]
    if not gcmd.get_int("START", 0):
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PLR_PROBE_TEST plan (no motion yet):\n"
            "  will probe %d times at X=%.3f Y=%.3f, descending at %.2f mm/s\n"
            "  (probe_speed from [plr]), retracting between samples\n"
            "  then stage probe_resolution for SAVE_CONFIG.\n"
            "Re-run with START=1 to consent to this motion."
            % (samples, pos[0], pos[1], probe_speed)
        )
        return
    # Mirror PROBE_ACCURACY's dummy per-sample command
    # (klippy/extras/probe.py:136-140), forcing SAMPLES=1 so each
    # run_probe takes exactly one sample, and pinning PROBE_SPEED to
    # the [plr] probe_speed tunable (get_probe_params reads it from
    # the gcode command, probe.py:299).
    fo_params = dict(gcmd.get_command_parameters())
    fo_params["SAMPLES"] = "1"
    fo_params["PROBE_SPEED"] = "%.3f" % (probe_speed,)
    fo_params.pop("START", None)
    gcode = printer.lookup_object("gcode")
    fo_gcmd = gcode.create_gcode_command("", "", fo_params)
    params = probe.get_probe_params(fo_gcmd)
    start_pos = toolhead.get_position()
    gcmd.respond_info(
        "PLR_PROBE_TEST at X:%.3f Y:%.3f (samples=%d probe_speed=%.2f "
        "lift_speed=%.2f retract=%.3f)"
        % (
            start_pos[0],
            start_pos[1],
            samples,
            probe_speed,
            params["lift_speed"],
            params["sample_retract_dist"],
        )
    )
    # Probe loop, as probe.py:142-153.
    probe_session = probe.start_probe_session(fo_gcmd)
    try:
        for _ in range(samples):
            probe_session.run_probe(fo_gcmd)
            lift_z = toolhead.get_position()[2] + params["sample_retract_dist"]
            toolhead.manual_move(
                [start_pos[0], start_pos[1], lift_z], params["lift_speed"]
            )
        positions = probe_session.pull_probed_results()
    finally:
        probe_session.end_probe_session()
    heights = [p.bed_z for p in positions]
    stats = compute_stats(heights)
    resolution = resolution_from_stddev(stats["stddev"])
    plugin.probe_resolution = resolution
    configfile = printer.lookup_object("configfile")
    configfile.set("plr", "probe_resolution", "%.6f" % (resolution,))
    plugin.note_pending_save("probe_resolution")
    gcmd.respond_info(
        "plr probe test: samples %d, range %.6f, stddev %.6f, median %.6f\n"
        "plr: probe_resolution = %.6f mm (max(2*stddev, %.3f))\n"
        "The SAVE_CONFIG command will update the printer config file\n"
        "with the above and restart the printer."
        % (
            samples,
            stats["range"],
            stats["stddev"],
            stats["median"],
            resolution,
            MIN_PROBE_RESOLUTION,
        )
    )
