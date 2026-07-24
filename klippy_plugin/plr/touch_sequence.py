"""Sliding-window consensus touch probing: the ``PLR_TOUCH`` command.

Ported from Cartographer3D's Survey Touch consensus sampler
(``src/cartographer/probe/touch_mode.py`` — ``run_probe_sequence``
lines 76-137, ``find_best_subset`` 180-190, ``compute_range`` 173-177,
``_perform_single_probe`` 256-281).  The pattern, not the code: this is
fresh house-style code, with a carto ``file:line`` citation wherever a
mechanism is adopted.

Why a sliding window at all — the anti-cherry-pick invariant.  A naive
"collect N touches, keep the tightest N-of-M subset" sampler will happily
assemble a passing subset from touches taken at completely different
times across a noisy run (a low reading early, another mid-run, another
late — physically unrelated).  Cartographer's fix (touch_mode.py:107-116)
is to only ever search the LAST ``samples + max_noisy`` touches: a passing
subset must be ``samples`` consistent touches within a short recent
window, so consistency has to be contemporaneous.  This module preserves
that exactly, and the property tests in ``tests/test_touch_sequence.py``
pin it: a stream whose good touches are scattered so no window ever holds
``samples`` of them MUST fail, even though a global search would "pass".

Layering:

* pure logic (``compute_range``, ``find_best_subset``, ``ConsensusConfig``,
  ``run_consensus``) is klippy-free and exhaustively unit-testable;
* the orchestrator ``perform_consensus_touch`` wraps a single klippy
  probe-session touch with the retract/accel safety invariants
  (carto touch_mode.py:256-281) and drives ``run_consensus``;
* ``cmd_PLR_TOUCH`` is the console command (gates + reporting), wired
  into klippy by :mod:`plr.plugin`.
"""

import collections
import logging
from dataclasses import dataclass
from itertools import combinations
from math import ceil, isfinite

from . import setup_checks
from .probe_test import _print_active

logger = logging.getLogger(__name__)

# --- consensus schema constants --------------------------------------
# Defaults and hard limits for the PLR_TOUCH consensus sampler.  These
# mirror carto's touch config schema (interfaces/configuration.py) with
# our own console-parameter names.
MIN_SAMPLES = 3
DEFAULT_SAMPLES = 3
DEFAULT_MAX_NOISY = 2
DEFAULT_MAX_SAMPLES = 10
MAX_MAX_SAMPLES = 20

# The window is samples + max_noisy touches (carto touch_mode.py:59
# max_window = touch.samples + touch.max_noisy_samples).  Capped so the
# combinatorial subset search below stays trivial — see find_best_subset.
MAX_WINDOW = 10

# Acceptable spread between the touches of a passing subset.  The HARD
# CAP is carto's own ceiling (configuration.py:248 — sample_range option
# is declared max=0.015): a larger value is a refusal, not a clamp,
# because a touch consensus looser than 15 microns is not a consensus
# worth trusting for recovery.
DEFAULT_SAMPLE_RANGE = 0.010
SAMPLE_RANGE_CAP = 0.015

# Retract between touches (carto retract_distance, configuration.py:247
# — default 2.0, min 1.0).  Guarantees a touch never begins its
# descent from below a safe standoff height.
DEFAULT_RETRACT = 2.0
MIN_RETRACT = 1.0

# Accel clamp applied around each descending touch (carto
# touch_mode.py:33 TOUCH_ACCEL = 100): a gentle accel keeps the trigger
# repeatable and the impact soft.
DEFAULT_TOUCH_ACCEL = 100.0
MIN_TOUCH_ACCEL = 50.0
MAX_TOUCH_ACCEL = 1000.0


class ConsensusError(RuntimeError):
    """Raised when a consensus sequence exhausts its touch budget.

    Carries every number the console message and the escalated retry
    command need, so the command layer never has to reconstruct them:
    how many consistent touches were wanted (``samples``), the spread
    limit (``sample_range``), the sliding ``window``, how many touches
    were actually taken (``touches``), and every sample value collected
    (``all_samples``).
    """

    def __init__(self, samples, sample_range, window, touches, all_samples):
        self.samples = samples
        self.sample_range = sample_range
        self.window = window
        self.touches = touches
        self.all_samples = tuple(all_samples)
        super().__init__(
            "unable to find %d touches within %s in a window of %d after "
            "%d touches" % (samples, format_distance(sample_range), window, touches)
        )


@dataclass(frozen=True)
class ConsensusConfig:
    """Validated parameters for one consensus touch sequence.

    Cross-field validation lives in ``__post_init__`` and REFUSES (raises
    ``ValueError``) rather than clamping, so a caller cannot silently get
    a looser-than-asked consensus.  The command layer converts the
    ``ValueError`` into a ``gcmd.error``.
    """

    samples: int = DEFAULT_SAMPLES
    max_samples: int = DEFAULT_MAX_SAMPLES
    max_noisy: int = DEFAULT_MAX_NOISY
    sample_range: float = DEFAULT_SAMPLE_RANGE

    def __post_init__(self):
        if self.samples < MIN_SAMPLES:
            raise ValueError(
                "SAMPLES=%d must be at least %d for a meaningful consensus"
                % (self.samples, MIN_SAMPLES)
            )
        if self.max_noisy < 0:
            raise ValueError("max_noisy=%d must not be negative" % (self.max_noisy,))
        if self.max_samples < self.samples:
            raise ValueError(
                "MAX_SAMPLES=%d must be >= SAMPLES=%d"
                % (self.max_samples, self.samples)
            )
        if self.max_samples > MAX_MAX_SAMPLES:
            raise ValueError(
                "MAX_SAMPLES=%d exceeds the maximum of %d touches per sequence"
                % (self.max_samples, MAX_MAX_SAMPLES)
            )
        window = self.samples + self.max_noisy
        if window > MAX_WINDOW:
            raise ValueError(
                "consensus window %d (SAMPLES=%d + max_noisy=%d) exceeds the "
                "maximum of %d; the subset search is only trivial for small "
                "windows" % (window, self.samples, self.max_noisy, MAX_WINDOW)
            )
        if not isfinite(self.sample_range) or self.sample_range <= 0.0:
            raise ValueError(
                "SAMPLE_RANGE=%s must be a positive distance" % (self.sample_range,)
            )
        if self.sample_range > SAMPLE_RANGE_CAP:
            # carto configuration.py:248 — sample_range is capped at
            # 0.015mm; a looser consensus is refused, never clamped.
            raise ValueError(
                "SAMPLE_RANGE=%s exceeds the hard cap of %.3fmm — a touch "
                "consensus looser than that is not trustworthy for recovery "
                "(cap from cartographer configuration.py:248)"
                % (format_distance(self.sample_range), SAMPLE_RANGE_CAP)
            )

    @property
    def window(self):
        """Sliding-window length: the most recent touches searched."""
        return self.samples + self.max_noisy


@dataclass(frozen=True)
class ConsensusResult:
    """Outcome of a successful consensus sequence.

    ``subset`` is the winning group of ``samples`` touches; ``median`` is
    its median (the reported trigger height); ``range`` is that subset's
    spread; ``touches_used`` is how many touches were taken to get there;
    ``all_samples`` is every touch collected, in order.
    """

    median: float
    range: float
    subset: tuple
    touches_used: int
    all_samples: tuple


# Parsed PLR_TOUCH / per-sequence parameters handed to the orchestrator.
TouchParams = collections.namedtuple(
    "TouchParams", ["config", "speed", "retract", "touch_accel"]
)


def _median(values):
    """Median of a non-empty sequence (pure python; numpy-free).

    numpy is optional at runtime for this plugin (README "Development"),
    so the median is computed directly rather than via ``np.median``.
    """
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def format_distance(distance_mm):
    """Format a distance so a nonzero spread never prints as ``0.000``.

    Rounds UP at the last displayed decimal (ceiling), so e.g. a real
    0.0004mm range shows as ``0.001`` rather than a falsely perfect
    ``0.000`` (carto calibrate.py:72-82 ``format_distance``).
    """
    if not isfinite(distance_mm):
        return "inf" if distance_mm > 0 else str(distance_mm)
    rounded = ceil(distance_mm * 1000) / 1000.0
    return "%.3f" % (rounded,)


def compute_range(samples):
    """Spread (max - min) of ``samples``; ``inf`` for fewer than two.

    carto touch_mode.py:173-177 ``compute_range``.  ``inf`` for a
    degenerate group makes it sort last in the subset search.
    """
    if len(samples) < 2:
        return float("inf")
    return max(samples) - min(samples)


def find_best_subset(window, size):
    """Minimal-range subset of ``size`` touches drawn from ``window``.

    carto touch_mode.py:180-190 ``find_best_subset`` (there via
    ``heapq.nsmallest(1, ...)``; a single linear scan is the same result
    without importing heapq).  Returns the winning subset as a tuple, or
    ``None`` when the window is too short.

    Complexity: C(len(window), size) subsets.  ``window`` is bounded to
    ``MAX_WINDOW`` (10) at config time, so the worst case is C(10, 5) =
    252 — trivial.  The bound is re-asserted here so a direct caller
    cannot smuggle in a window that would make the combinatorics blow up.
    """
    if len(window) > MAX_WINDOW:
        raise ValueError(
            "window of %d exceeds the maximum of %d; subset search is only "
            "kept trivial for small windows" % (len(window), MAX_WINDOW)
        )
    if size > len(window):
        return None
    best = None
    best_range = float("inf")
    for combo in combinations(window, size):
        spread = compute_range(combo)
        if spread < best_range:
            best_range = spread
            best = combo
    return best


def run_consensus(probe_once, config, log=None):
    """Collect touches one at a time until a recent subset agrees.

    The Cartographer loop (touch_mode.py:100-137): take one touch per
    iteration; once at least ``samples`` touches exist, search ONLY the
    sliding window of the most recent ``config.window`` touches for a
    subset of ``samples`` whose spread is within ``sample_range``.  The
    window is what defeats cherry-picking — a passing subset must be
    ``samples`` mutually-consistent touches taken close together in time,
    not assembled from across a noisy run.

    Returns a :class:`ConsensusResult` on the first accepted subset;
    raises :class:`ConsensusError` (carrying every number the caller
    needs) once ``max_samples`` touches have been taken without one.

    ``log`` is an optional ``callable(str)`` for per-touch diagnostics.
    """
    if log is None:
        log = _noop_log
    collected = []
    for i in range(config.max_samples):
        pos = probe_once()
        collected.append(pos)
        log("touch %d: %.6f" % (i + 1, pos))

        if len(collected) < config.samples:
            continue

        window = collected[-config.window :]
        best = find_best_subset(window, config.samples)
        if best is None:
            continue
        best_range = compute_range(best)
        if best_range > config.sample_range:
            continue

        median = _median(best)
        log(
            "consensus: subset (%s) range %s median %.6f"
            % (
                ", ".join("%.6f" % (s,) for s in best),
                format_distance(best_range),
                median,
            )
        )
        return ConsensusResult(
            median=median,
            range=best_range,
            subset=tuple(best),
            touches_used=len(collected),
            all_samples=tuple(collected),
        )

    raise ConsensusError(
        samples=config.samples,
        sample_range=config.sample_range,
        window=config.window,
        touches=len(collected),
        all_samples=collected,
    )


def _noop_log(_message):
    """Default ``log`` sink for :func:`run_consensus`."""
    return None


def perform_consensus_touch(plugin, gcmd_params):
    """Run one consensus sequence at the current XY with safety invariants.

    Wraps a single klippy probe-session touch (the same
    ``start_probe_session`` / ``run_probe`` / ``pull_probed_results``
    mechanics :mod:`plr.probe_test` uses) with the three per-touch
    invariants Cartographer applies in ``_perform_single_probe``
    (touch_mode.py:256-281), in this exact order:

    (a) retract-before-arm (carto touch_mode.py:258-260): if the toolhead
        is below the retract height, lift to it at lift speed and
        ``wait_moves`` before arming — a descent never begins from below
        a safe standoff.
    (b) accel clamp (carto touch_mode.py:262-274): read the current max
        accel, clamp to ``touch_accel`` for the descent, and restore the
        prior accel in a ``finally`` so it is restored on EVERY path,
        including a probe that raises mid-descent.
    (c) retract-after-trigger (carto touch_mode.py:276-281): lift to
        ``max(trigger + retract, retract)`` at lift speed, so the next
        touch again starts from a safe height.

    Returns the :class:`ConsensusResult`; propagates
    :class:`ConsensusError` on budget exhaustion.
    """
    printer = plugin.printer
    reactor = printer.get_reactor()
    toolhead = printer.lookup_object("toolhead")
    probe = printer.lookup_object("probe")
    gcode = printer.lookup_object("gcode")

    config = gcmd_params.config
    retract = gcmd_params.retract
    touch_accel = gcmd_params.touch_accel

    # Force a single-sample descending probe per touch, pinning the
    # descent speed, exactly as PLR_PROBE_TEST builds its dummy command
    # (probe_test / klippy probe.py:136-140).
    fo_params = {"SAMPLES": "1", "PROBE_SPEED": "%.3f" % (gcmd_params.speed,)}
    fo_gcmd = gcode.create_gcode_command("", "", fo_params)
    lift_speed = probe.get_probe_params(fo_gcmd)["lift_speed"]

    start_pos = toolhead.get_position()
    session = probe.start_probe_session(fo_gcmd)

    def probe_once():
        # (a) retract-before-arm (carto touch_mode.py:258-260)
        if toolhead.get_position()[2] < retract:
            toolhead.manual_move([start_pos[0], start_pos[1], retract], lift_speed)
            toolhead.wait_moves()
        # (b) accel clamp (carto touch_mode.py:262-274) — restore on every
        # path, including a probe that raises, via the finally.
        prior_accel = toolhead.get_status(reactor.monotonic())["max_accel"]
        toolhead.set_max_velocities(None, touch_accel, None, None)
        try:
            session.run_probe(fo_gcmd)
            trigger = session.pull_probed_results()[-1].bed_z
        finally:
            toolhead.set_max_velocities(None, prior_accel, None, None)
        # (c) retract-after-trigger (carto touch_mode.py:276-281)
        target = max(trigger + retract, retract)
        toolhead.manual_move([start_pos[0], start_pos[1], target], lift_speed)
        return trigger

    try:
        result = run_consensus(probe_once, config, log=logger.debug)
        toolhead.wait_moves()
    finally:
        session.end_probe_session()
    return result


# --- console command -------------------------------------------------


def require_touch_ready(plugin, gcmd, command):
    """Shared PLR_TOUCH / PLR_PROBE_TEST gates: refuse loudly, no motion.

    Same signals PLR_PROBE_TEST already uses: not a descending probe
    method, no active print, a cool nozzle (the shared contact-operation
    temperature gate, setup_checks.require_nozzle_cool), xyz homed, and a
    probe object present.  Returns ``(toolhead, probe)`` when every gate
    passes.
    """
    printer = plugin.printer
    if plugin.probe_method == "adxl_drag":
        raise gcmd.error(
            "%s needs a descending probe (probe_method tap or load_cell); "
            "probe_method is adxl_drag — use PLR_DRAG_PROBE instead" % (command,)
        )
    if _print_active(printer):
        raise gcmd.error(
            "%s refused: a print is active (it moves the toolhead); wait for "
            "the print to finish or cancel it" % (command,)
        )
    setup_checks.require_nozzle_cool(plugin, gcmd, command)
    toolhead = printer.lookup_object("toolhead")
    eventtime = printer.get_reactor().monotonic()
    homed = toolhead.get_status(eventtime)["homed_axes"]
    if any(axis not in homed for axis in "xyz"):
        raise gcmd.error(
            "%s refused: printer must be homed (homed axes: '%s', need xyz) — "
            "run G28 first" % (command, homed)
        )
    probe = printer.lookup_object("probe", None)
    if probe is None:
        raise gcmd.error(
            "no probe object — is the [%s] section present?"
            % ("probe" if plugin.probe_method == "tap" else "load_cell_probe")
        )
    return toolhead, probe


def parse_touch_params(gcmd, default_speed):
    """Parse the shared consensus parameters into a :class:`TouchParams`.

    Field-level bounds klippy owns naturally (SAMPLES floor, RETRACT
    floor, TOUCH_ACCEL range) are enforced by ``gcmd`` so the console
    error is klippy's standard text.  The cross-field and hard-cap rules
    (MAX_SAMPLES ceiling, window bound, SAMPLE_RANGE cap) are deferred to
    :class:`ConsensusConfig` so they are refusals with a naming message —
    hence MAX_SAMPLES / SAMPLE_RANGE are parsed WITHOUT a ``maxval`` here.
    Raises ``ValueError`` (from ConsensusConfig) on a refused combination.
    """
    samples = gcmd.get_int("SAMPLES", DEFAULT_SAMPLES, minval=MIN_SAMPLES)
    max_samples = gcmd.get_int("MAX_SAMPLES", DEFAULT_MAX_SAMPLES, minval=MIN_SAMPLES)
    sample_range = gcmd.get_float("SAMPLE_RANGE", DEFAULT_SAMPLE_RANGE, above=0.0)
    speed = gcmd.get_float("SPEED", default_speed, above=0.0)
    retract = gcmd.get_float("RETRACT", DEFAULT_RETRACT, minval=MIN_RETRACT)
    touch_accel = gcmd.get_float(
        "TOUCH_ACCEL",
        DEFAULT_TOUCH_ACCEL,
        minval=MIN_TOUCH_ACCEL,
        maxval=MAX_TOUCH_ACCEL,
    )
    config = ConsensusConfig(
        samples=samples,
        max_samples=max_samples,
        max_noisy=DEFAULT_MAX_NOISY,
        sample_range=sample_range,
    )
    return TouchParams(
        config=config, speed=speed, retract=retract, touch_accel=touch_accel
    )


def format_command(name, pairs):
    """Render ``name`` plus ``(KEY, value_str)`` pairs as one g-code line.

    Used to embed a copy-pasteable retry command in failure text; the
    output round-trips through klippy's ``KEY=VALUE`` parser.
    """
    parts = [name]
    for key, value in pairs:
        parts.append("%s=%s" % (key, value))
    return " ".join(parts)


def touch_param_pairs(params, overrides=None):
    """The consensus parameters as ordered ``(KEY, value_str)`` pairs.

    ``overrides`` (a dict) replaces individual formatted values — used to
    show an escalated MAX_SAMPLES in a retry command.
    """
    overrides = overrides or {}
    config = params.config
    pairs = [
        ("SAMPLES", "%d" % (config.samples,)),
        ("MAX_SAMPLES", "%d" % (config.max_samples,)),
        ("SAMPLE_RANGE", "%.3f" % (config.sample_range,)),
        ("SPEED", "%.2f" % (params.speed,)),
        ("RETRACT", "%.2f" % (params.retract,)),
        ("TOUCH_ACCEL", "%.0f" % (params.touch_accel,)),
    ]
    return [(key, overrides.get(key, value)) for key, value in pairs]


def escalated_max_samples(max_samples):
    """MAX_SAMPLES for a retry: 1.5x (ceil), capped at the hard maximum.

    carto calibrate.py:398-412 escalates the search bound by 1.5x in its
    failure hint; the cap keeps the suggestion within what
    :class:`ConsensusConfig` would accept.
    """
    return min(MAX_MAX_SAMPLES, int(ceil(max_samples * 1.5)))


def consensus_failure_text(command, error, params):
    """Console message for a :class:`ConsensusError`, with a retry line.

    Names the acceptance criteria that failed (how many consistent
    touches within what spread, in what window, after how many touches)
    and offers a copy-pasteable retry with MAX_SAMPLES escalated 1.5x
    (carto calibrate.py:398-412).
    """
    new_max = escalated_max_samples(params.config.max_samples)
    retry = format_command(
        command,
        [("START", "1")]
        + touch_param_pairs(params, overrides={"MAX_SAMPLES": "%d" % (new_max,)}),
    )
    return (
        "%s failed: could not find %d touches within %s of each other in a "
        "sliding window of %d, after %d touches.\n"
        "  samples taken: [%s]\n"
        "Retry with a larger touch budget:\n  %s"
        % (
            command,
            error.samples,
            format_distance(error.sample_range),
            error.window,
            error.touches,
            ", ".join("%.6f" % (s,) for s in error.all_samples),
            retry,
        )
    )


def cmd_PLR_TOUCH(plugin, gcmd):
    """PLR_TOUCH [SAMPLES=] [MAX_SAMPLES=] [SAMPLE_RANGE=] [SPEED=]
    [RETRACT=] [TOUCH_ACCEL=] — one consensus touch at the current XY."""
    try:
        params = parse_touch_params(gcmd, plugin.tunables["probe_speed"])
    except ValueError as e:
        raise gcmd.error("PLR_TOUCH: %s" % (e,)) from None

    toolhead, _probe = require_touch_ready(plugin, gcmd, "PLR_TOUCH")
    start_pos = toolhead.get_position()
    gcmd.respond_info(
        "PLR_TOUCH consensus at X:%.3f Y:%.3f (want %d touches within %s, "
        "window %d, budget %d, speed %.2f)"
        % (
            start_pos[0],
            start_pos[1],
            params.config.samples,
            format_distance(params.config.sample_range),
            params.config.window,
            params.config.max_samples,
            params.speed,
        )
    )

    try:
        result = perform_consensus_touch(plugin, params)
    except ConsensusError as e:
        plugin.last_touch_result = None
        raise gcmd.error(consensus_failure_text("PLR_TOUCH", e, params)) from None

    plugin.last_touch_result = {
        "median_z": result.median,
        "range": result.range,
        "samples_used": params.config.samples,
        "touches": result.touches_used,
    }
    logger.debug(
        "PLR_TOUCH samples: [%s]; subset: [%s]",
        ", ".join("%.6f" % (s,) for s in result.all_samples),
        ", ".join("%.6f" % (s,) for s in result.subset),
    )
    gcmd.respond_info(
        "PLR_TOUCH: median %.6f, range %s, min %.6f, max %.6f\n"
        "  %d touches used of %d (window %d, limit %s)"
        % (
            result.median,
            format_distance(result.range),
            min(result.subset),
            max(result.subset),
            result.touches_used,
            params.config.max_samples,
            params.config.window,
            format_distance(params.config.sample_range),
        )
    )
