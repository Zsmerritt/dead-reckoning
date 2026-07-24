"""Probe repeatability diagnostic: the PLR_PROBE_TEST command.

Two-tier verification, ported from Cartographer3D's touch-calibrate
verifier (``src/cartographer/macros/touch/calibrate.py`` —
``ThresholdVerifier.verify`` lines 156-223, its early-exit 205-215, the
``format_distance`` ceiling-round 72-82, and the escalating failure hint
398-412).  The pattern, not the code.

Where the OLD PLR_PROBE_TEST ran N single descending probes and reported
their raw spread, the new one runs ``SEQUENCES`` FULL consensus touch
sequences (each a :func:`plr.touch_sequence.perform_consensus_touch`, the
same sliding-window sampler PLR_TOUCH uses) and checks that the resulting
per-sequence medians agree:

* acceptance = range of the sequence medians <= ``VERIFY_RANGE``
  (default 2x the per-sequence ``SAMPLE_RANGE``, capped at 4x; a
  VERIFY_RANGE below SAMPLE_RANGE or above 4x is refused loudly, carto
  calibrate.py:269-274);
* early-exit the moment the running median range exceeds the limit —
  no point taking more sequences once inconsistent (carto
  calibrate.py:205-215);
* on success, stage ``probe_resolution`` for SAVE_CONFIG (see
  :func:`resolution_from_medians` for the formula);
* on failure, a copy-pasteable retry with SEQUENCES escalated and
  VERIFY_RANGE loosened (capped).

Safety gates are unchanged and all client-side, all before motion:
refuses adxl_drag (no descending probe), refuses while a print is
active, refuses unless xyz are homed, and does NOTHING without an
explicit ``START=1`` (motion consent).  The per-touch retract/accel
invariants live in :func:`plr.touch_sequence.perform_consensus_touch`.
"""

from . import calibration_meta

# Verification tier: how many full consensus sequences to run.
MIN_SEQUENCES = 3
DEFAULT_SEQUENCES = 5
MAX_SEQUENCES = 10

# Floor for the persisted probe_resolution (mm).  A perfectly repeating
# probe still cannot resolve below roughly a microstep; claiming better
# than 5 microns would let the planner trust noise.
MIN_PROBE_RESOLUTION = 0.005


def compute_stats(values):
    """Range, population standard deviation, median and mean of ``values``.

    The statistics PROBE_ACCURACY prints (klippy/extras/probe.py:154-164
    computes range/average/median and a population sigma).
    """
    if not values:
        raise ValueError("compute_stats: no samples")
    n = len(values)
    mean = sum(values) / n
    range_value = max(values) - min(values)
    sigma = (sum((v - mean) ** 2 for v in values) / n) ** 0.5
    ordered = sorted(values)
    mid = n // 2
    if n % 2:
        median = ordered[mid]
    else:
        median = (ordered[mid - 1] + ordered[mid]) / 2.0
    return {"range": range_value, "stddev": sigma, "median": median, "mean": mean}


def resolution_from_medians(median_range, stddev, minimum=MIN_PROBE_RESOLUTION):
    """probe_resolution staged for SAVE_CONFIG from the sequence medians.

    ``max(2*stddev, median_range/2, minimum)`` — the trust radius plrd
    uses for recovery probing, taken as the loosest of three honest
    estimates of how well the probe repeats:

    * ``2*stddev`` — a two-sigma spread of the sequence medians (the same
      2-sigma basis the old single-probe resolution used);
    * ``median_range/2`` — half the observed peak-to-peak of the medians,
      a floor that respects the worst swing actually seen even when a
      small sample count makes stddev look optimistic;
    * ``minimum`` (0.005mm) — the microstep floor: no probe resolves
      finer than this regardless of how still the medians sat.
    """
    return max(2.0 * stddev, median_range / 2.0, minimum)


def _print_active(printer):
    """True when a print is running/paused; None-safe on bare configs.

    Prefers print_stats (klippy/extras/print_stats.py:118 exposes
    'state' in {standby, printing, paused, complete, error, cancelled};
    only exists with [virtual_sdcard]).  Falls back to idle_timeout
    (klippy/extras/idle_timeout.py:34-40: state 'Printing' means motion
    was commanded within the last idle window — coarser, so a recent
    manual move can also read as active; documented caveat).

    Shared with the drag oracle (drag_probe._print_active import) and the
    touch commands (touch_sequence.require_touch_ready).
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


def _verify_failure_text(ts, stats, sequences, verify_range, sample_range, params):
    """Console message for an inconsistent-medians failure, with a retry.

    Reports a PROBE_ACCURACY-style block of the sequence medians and a
    copy-pasteable retry with SEQUENCES escalated 1.5x (cap MAX_SEQUENCES)
    and VERIFY_RANGE loosened 1.5x (cap 4x SAMPLE_RANGE) — the two knobs
    that buy consistency headroom (carto calibrate.py:398-412).
    """
    from math import ceil

    new_sequences = min(MAX_SEQUENCES, int(ceil(sequences * 1.5)))
    new_verify = min(4.0 * sample_range, verify_range * 1.5)
    retry = ts.format_command(
        "PLR_PROBE_TEST",
        [
            ("START", "1"),
            ("SEQUENCES", "%d" % (new_sequences,)),
            ("VERIFY_RANGE", "%.3f" % (new_verify,)),
        ]
        + ts.touch_param_pairs(params),
    )
    return (
        "PLR_PROBE_TEST failed: sequence medians disagree.\n"
        "  sequences %d, range %s (limit %s), stddev %s, median %.6f\n"
        "  medians: [%s]\n"
        "The probe does not repeat within VERIFY_RANGE. Retry with more "
        "sequences and a looser (capped) limit:\n  %s"
        % (
            len(stats["_medians"]),
            ts.format_distance(stats["range"]),
            ts.format_distance(verify_range),
            ts.format_distance(stats["stddev"]),
            stats["median"],
            ", ".join("%.6f" % (m,) for m in stats["_medians"]),
            retry,
        )
    )


def cmd_PLR_PROBE_TEST(plugin, gcmd):
    """PLR_PROBE_TEST [SEQUENCES=5] [VERIFY_RANGE=] [SAMPLES=] [MAX_SAMPLES=]
    [SAMPLE_RANGE=] [SPEED=] [RETRACT=] [TOUCH_ACCEL=] START=1."""
    # Lazy import breaks the probe_test <-> touch_sequence cycle
    # (touch_sequence imports _print_active from this module at load).
    from . import touch_sequence as ts

    printer = plugin.printer
    try:
        params = ts.parse_touch_params(gcmd, plugin.tunables["probe_speed"])
    except ValueError as e:
        raise gcmd.error("PLR_PROBE_TEST: %s" % (e,)) from None
    sample_range = params.config.sample_range

    sequences = gcmd.get_int(
        "SEQUENCES", DEFAULT_SEQUENCES, minval=MIN_SEQUENCES, maxval=MAX_SEQUENCES
    )
    # VERIFY_RANGE is parsed WITHOUT gcmd bounds so the relational checks
    # (>= SAMPLE_RANGE, <= 4x SAMPLE_RANGE) are our own loud refusals
    # rather than klippy's generic min/max text (carto calibrate.py:269-274).
    verify_range = gcmd.get_float("VERIFY_RANGE", 2.0 * sample_range, above=0.0)
    if verify_range < sample_range:
        raise gcmd.error(
            "PLR_PROBE_TEST: VERIFY_RANGE=%s must be >= SAMPLE_RANGE=%s — a "
            "verification tighter than a single sequence's own consensus is "
            "self-contradictory"
            % (ts.format_distance(verify_range), ts.format_distance(sample_range))
        )
    if verify_range > 4.0 * sample_range:
        raise gcmd.error(
            "PLR_PROBE_TEST: VERIFY_RANGE=%s must be <= 4x SAMPLE_RANGE (%s) — "
            "a looser bound would rubber-stamp a probe that does not repeat"
            % (
                ts.format_distance(verify_range),
                ts.format_distance(4.0 * sample_range),
            )
        )

    toolhead, _probe = ts.require_touch_ready(plugin, gcmd, "PLR_PROBE_TEST")

    if not gcmd.get_int("START", 0):
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PLR_PROBE_TEST plan (no motion yet):\n"
            "  will run up to %d consensus sequences at X=%.3f Y=%.3f\n"
            "  (each: %d touches within %s in a window of %d, up to %d touches),\n"
            "  accept if the sequence medians agree within %s,\n"
            "  then stage probe_resolution for SAVE_CONFIG.\n"
            "Re-run with START=1 to consent to this motion."
            % (
                sequences,
                pos[0],
                pos[1],
                params.config.samples,
                ts.format_distance(sample_range),
                params.config.window,
                params.config.max_samples,
                ts.format_distance(verify_range),
            )
        )
        return

    # Refuse up front (before any motion) when the calibration cannot be
    # stamped — a stamped-or-nothing policy (calibration_meta), so a probe
    # test never runs its sequences only to fail to persist the result.
    calibration_meta.require_klipper_version(printer, gcmd.error)

    start_pos = toolhead.get_position()
    gcmd.respond_info(
        "PLR_PROBE_TEST at X:%.3f Y:%.3f: %d consensus sequences, accept "
        "median range <= %s (probe_speed=%.2f)"
        % (
            start_pos[0],
            start_pos[1],
            sequences,
            ts.format_distance(verify_range),
            params.speed,
        )
    )

    medians = []
    for i in range(sequences):
        try:
            result = ts.perform_consensus_touch(plugin, params)
        except ts.ConsensusError as e:
            # A single sequence could not even reach consensus: the probe
            # is too noisy touch-to-touch.  Surface the consensus criteria
            # and a retry that grows the per-sequence touch budget.
            raise gcmd.error(
                "PLR_PROBE_TEST: sequence %d/%d could not reach consensus.\n%s"
                % (
                    i + 1,
                    sequences,
                    ts.consensus_failure_text("PLR_PROBE_TEST", e, params),
                )
            ) from None
        medians.append(result.median)
        # Early exit once inconsistent (carto calibrate.py:205-215).
        if len(medians) >= 2 and (max(medians) - min(medians)) > verify_range:
            break

    stats = compute_stats(medians)
    stats["_medians"] = list(medians)

    if stats["range"] > verify_range:
        raise gcmd.error(
            _verify_failure_text(
                ts, stats, sequences, verify_range, sample_range, params
            )
        )

    resolution = resolution_from_medians(stats["range"], stats["stddev"])
    # Stage probe_resolution together with its version/fingerprint stamps,
    # atomically (nothing is written if the Klipper version is unavailable —
    # already guarded above, re-checked here).
    try:
        calibration_meta.stage_calibration(
            plugin,
            calibration_meta.GROUP_PROBE_RESOLUTION,
            [("probe_resolution", "%.6f" % (resolution,))],
        )
    except calibration_meta.UnstampableError as e:
        raise gcmd.error("PLR_PROBE_TEST: %s" % (e,)) from None
    plugin.probe_resolution = resolution
    gcmd.respond_info(
        "plr probe test: %d sequences, median range %s (limit %s), "
        "stddev %s, median %.6f\n"
        "plr: probe_resolution = %.6f mm (max(2*stddev, median_range/2, %.3f))\n"
        "The SAVE_CONFIG command will update the printer config file\n"
        "with the above and restart the printer."
        % (
            len(medians),
            ts.format_distance(stats["range"]),
            ts.format_distance(verify_range),
            ts.format_distance(stats["stddev"]),
            stats["median"],
            resolution,
            MIN_PROBE_RESOLUTION,
        )
    )
