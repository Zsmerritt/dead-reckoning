#!/usr/bin/env python3
"""Regenerates `fixtures/real/realistic_orca.gcode`.

# Why this file exists

`fixtures/real/` is auto-discovered by `plr-gcode`'s `real_corpus_full_pipeline`
test, and **that test passed vacuously from the day it was written until this
fixture landed** -- the directory held only a README, so the loop body never
ran. Nobody noticed, because a vacuous test is indistinguishable from a
passing one in every report. That is the specific failure this file closes,
and it is worth stating plainly: the suite had the shape of coverage without
the substance.

# Why the geometry is synthetic

Real sliced files were captured off the operator's printer to measure move and
E density (see below), and the operator ruled explicitly that their own design
files must not enter this repository's permanent history. So this fixture is
built the same way the `*_real_footer` fixtures in `fixtures/synthetic/` are,
which is the established precedent here:

    synthetic header + synthetic body + a REAL slicer footer, verbatim

The footer is not re-captured. It is copied from the already-committed
`fixtures/synthetic/orca_real_footer.gcode`, which `footer_generator.py`
already scrubbed and vetted -- so this adds **zero** new content derived from
anyone's model. The header is written from scratch to OrcaSlicer's shape.

## No THUMBNAIL_BLOCK, and it must stay that way

Real OrcaSlicer output opens with a `THUMBNAIL_BLOCK` holding a base64 PNG.
That PNG is **a rendering of the part**, so adding one would reintroduce
exactly the content the operator ruled out -- more directly than the g-code
does, since it is a picture rather than coordinates.

This is written down because the omission looks like a gap. A future reader
comparing this fixture against a real file will notice the missing block and
reasonably wonder whether the corpus should carry one for fidelity: to
exercise the parser against a very long base64 comment run, say. The answer is
no, and the reason is not fidelity but provenance. If a thumbnail-shaped input
is ever genuinely needed, synthesize the base64 payload (random bytes, or an
image of nothing) rather than copying a real block -- the parser cannot tell
the difference and the repository history can.

# What makes the body realistic rather than a sketch

Measured over four real OrcaSlicer 2.4.2 files (0.2 mm and 0.4 mm layers, ABS
and ASA), 114k lines in the largest:

    per-line positive E:  median 0.0537 mm   p10 0.0280   p90 0.0783
    retract lines:        419 of 114,107  (0.37%)

The generator reproduces that: extrusion is computed from real filament
geometry (0.2 mm layer, 0.42 mm extrusion width, 1.75 mm filament ->
0.0349 mm of filament per mm of travel), so a typical 1.5 mm chord carries
~0.052 mm and the distribution lands in the measured band rather than on a
round number. Travels carry a retract/Z-hop/unretract triple at a comparable
rate.

This matters beyond making the suite bite. `plrd`'s end-to-end fixture
deliberately puts 0.65 mm of E on single lines to keep stop-point matching
unambiguous, which means it cannot answer questions about evidence-interval
width at realistic density -- and it sits at exactly 8 of 8 candidates against
`ambiguity_limit`, so it has no headroom to answer them with. This fixture is
the harness that does have headroom.

# Usage

    python3 fixtures/real/realistic_generator.py

Deterministic: no RNG, so regenerating produces byte-identical output and a
diff means a real change.
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
FOOTER_SRC = os.path.join(HERE, "..", "synthetic", "orca_real_footer.gcode")
OUT = os.path.join(HERE, "realistic_orca.gcode")

# --- extrusion model (real filament geometry, not a round number) ---------
LAYER_H = 0.2
WIDTH = 0.42
FIL_D = 1.75
# mm of filament per mm of travel: (layer x width) / filament cross-section.
E_PER_MM = (LAYER_H * WIDTH) / (math.pi * (FIL_D / 2.0) ** 2)

LAYERS = 24
CENTER = (110.0, 110.0)
# Rounded rectangle, in mm. Two perimeters plus sparse infill.
HALF_X, HALF_Y, CORNER = 20.0, 14.0, 4.0
PERIM_SPACING = 0.42
INFILL_SPACING = 2.4
CHORD = 1.5  # nominal segment length, mm -> ~0.052 mm E per line

# Real slicer output does NOT emit one segment length: chord length tracks
# curvature and feature size, so per-line E spreads over roughly p10 0.028 to
# p90 0.078 mm. A single CHORD gives a degenerate distribution (measured:
# p10 == p90 == median), which would make the fixture useless for exactly the
# evidence-width questions it exists to answer. This deterministic cycle of
# target lengths reproduces the measured spread: 0.80 mm -> 0.028 mm E at the
# low end, 2.24 mm -> 0.078 mm at the high end.
CHORD_CYCLE = (0.80, 1.15, 1.50, 1.15, 2.24, 1.50, 0.95, 1.85, 1.30, 2.05)


def chord_lengths(total, phase):
    """Segment lengths tiling `total`, cycling CHORD_CYCLE from `phase`.

    Deterministic in `phase` so regeneration is byte-stable. The final
    segment absorbs the remainder, so the tiling is exact.
    """
    out, used, i = [], 0.0, phase
    while total - used > 1e-9:
        want = CHORD_CYCLE[i % len(CHORD_CYCLE)]
        i += 1
        if total - used - want < 0.3:  # avoid a degenerate tail segment
            out.append(total - used)
            break
        out.append(want)
        used += want
    return out


FEED_PERIM = 1800
FEED_INFILL = 2400
FEED_TRAVEL = 9000
RETRACT = 0.8
ZHOP = 0.4


def rounded_rect(inset):
    """Points of a rounded rectangle inset by `inset`, chorded at ~CHORD."""
    hx, hy = HALF_X - inset, HALF_Y - inset
    r = max(0.2, CORNER - inset)
    pts = []
    # Four straight runs and four corner arcs, counter-clockwise.
    sides = [
        ((-hx + r, -hy), (hx - r, -hy), (hx - r, -hy + r), -math.pi / 2, 0.0),
        ((hx, -hy + r), (hx, hy - r), (hx - r, hy - r), 0.0, math.pi / 2),
        ((hx - r, hy), (-hx + r, hy), (-hx + r, hy - r), math.pi / 2, math.pi),
        ((-hx, hy - r), (-hx, -hy + r), (-hx + r, -hy + r), math.pi, 1.5 * math.pi),
    ]
    phase = 0
    for (x0, y0), (x1, y1), (cx, cy), a0, a1 in sides:
        seg = math.hypot(x1 - x0, y1 - y0)
        acc = 0.0
        for step in chord_lengths(seg, phase):
            acc += step
            t = acc / seg
            pts.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
            phase += 1
        arc = abs(a1 - a0) * r
        acc = 0.0
        for step in chord_lengths(arc, phase):
            acc += step
            a = a0 + (a1 - a0) * (acc / arc)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
            phase += 1
    return pts


def emit_path(out, pts, start, feed, first_feed=True):
    """Extruding moves along `pts` starting from `start`. Returns end point."""
    x, y = start
    for i, (px, py) in enumerate(pts):
        d = math.hypot(px - x, py - y)
        if d < 1e-9:
            continue
        e = d * E_PER_MM
        f = f" F{feed}" if i == 0 and first_feed else ""
        out.append(f"G1 X{CENTER[0] + px:.3f} Y{CENTER[1] + py:.3f} E{e:.5f}{f}")
        x, y = px, py
    return (x, y)


def travel(out, dest, z):
    """Retract / Z-hop / move / un-hop / unretract -- the real travel shape."""
    out.append(f"G1 E-{RETRACT:.5f} F2100")
    out.append(f"G1 Z{z + ZHOP:.3f} F{FEED_TRAVEL}")
    out.append(f"G1 X{CENTER[0] + dest[0]:.3f} Y{CENTER[1] + dest[1]:.3f}")
    out.append(f"G1 Z{z:.3f}")
    out.append(f"G1 E{RETRACT:.5f} F2100")


def header():
    return [
        "; HEADER_BLOCK_START",
        (
            "; generated by OrcaSlicer 2.4.2 (synthetic fixture; see"
            " realistic_generator.py)"
        ),
        f"; total layer number: {LAYERS}",
        "; filament_density: 1.04",
        "; filament_diameter: 1.75",
        f"; max_z_height: {LAYERS * LAYER_H:.2f}",
        "; nozzle_diameter: 0.4",
        "; HEADER_BLOCK_END",
        "",
        "; no THUMBNAIL_BLOCK on purpose: in a real file it is a base64 PNG",
        "; rendering of the part, which is the geometry this fixture must not",
        "; carry. See the module docstring in realistic_generator.py.",
        "",
        "M104 S255",
        "M140 S100",
        "G90",
        "M83",
        "G28",
        "M190 S100",
        "M109 S255",
        (
            "EXCLUDE_OBJECT_DEFINE NAME=fixture_block.stl_id_0_copy_0"
            " CENTER=110,110 POLYGON=[[90,96],[130,96],[130,124],[90,124]]"
        ),
        "M204 S3000",
        f"G1 Z{LAYER_H:.3f} F7200",
        "G92 E0",
    ]


def footer():
    """The real OrcaSlicer footer, verbatim, from the committed fixture."""
    with open(FOOTER_SRC, encoding="utf-8") as fh:
        lines = fh.read().split("\n")
    # The committed fixture is synthetic-header + real-footer; the real part
    # begins at the object-stop marker the slicer writes after the last
    # deposition. Take everything from there.
    for i, line in enumerate(lines):
        if line.startswith("; stop printing object"):
            return lines[i:]
    raise SystemExit("footer marker not found in " + FOOTER_SRC)


def main():
    out = header()
    out.append("; EXECUTABLE_BLOCK_START")
    out.append("EXCLUDE_OBJECT_START NAME=fixture_block.stl_id_0_copy_0")
    pos = (0.0, 0.0)
    for layer in range(LAYERS):
        z = LAYER_H * (layer + 1)
        out.append(f";LAYER_CHANGE\n;Z:{z:.3f}")
        out.append("; CHANGE_LAYER")
        if layer:
            out.append(f"G1 Z{z:.3f} F7200")
        # Two perimeters, outer first.
        for p, inset in enumerate((0.0, PERIM_SPACING)):
            pts = rounded_rect(inset)
            out.append(";TYPE:Outer wall" if p == 0 else ";TYPE:Inner wall")
            if p == 0:
                travel(out, pts[-1], z)
            else:
                # Wall-to-wall: a plain non-extruding hop, no retract. Real
                # slicers do not retract for a sub-millimetre step, and
                # retracting here would put the fixture's retract rate at
                # ~0.6% against the 0.37% measured on real output.
                out.append(
                    f"G1 X{CENTER[0] + pts[-1][0]:.3f}"
                    f" Y{CENTER[1] + pts[-1][1]:.3f} F{FEED_TRAVEL}"
                )
            pos = emit_path(out, pts, pts[-1], FEED_PERIM)
        # Sparse infill: alternating diagonal-ish raster, direction per layer.
        out.append(";TYPE:Sparse infill")
        hx = HALF_X - 2 * PERIM_SPACING - 0.2
        hy = HALF_Y - 2 * PERIM_SPACING - 0.2
        n = int((2 * hy) / INFILL_SPACING)
        flip = layer % 2 == 1
        first = True
        for i in range(n):
            y = -hy + i * INFILL_SPACING
            a, b = (-hx, hx) if (i % 2 == 0) != flip else (hx, -hx)
            if first:
                travel(out, (a, y), z)
                first = False
                pos = (a, y)
            seg = []
            d = abs(b - a)
            acc = 0.0
            for step in chord_lengths(d, i):
                acc += step
                seg.append((a + (b - a) * (acc / d), y))
            pos = emit_path(out, seg, pos, FEED_INFILL, first_feed=(i == 0))
            if i + 1 < n:
                ny = -hy + (i + 1) * INFILL_SPACING
                d = math.hypot(0.0, ny - y)
                out.append(
                    f"G1 X{CENTER[0] + pos[0]:.3f} Y{CENTER[1] + ny:.3f}"
                    f" E{d * E_PER_MM:.5f}"
                )
                pos = (pos[0], ny)
    out.append("EXCLUDE_OBJECT_END NAME=fixture_block.stl_id_0_copy_0")
    out.append("; EXECUTABLE_BLOCK_END")
    out.extend(footer())

    text = "\n".join(out)
    if not text.endswith("\n"):
        text += "\n"
    with open(OUT, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)

    # Report the measured density so a regeneration that drifts is visible.
    import re

    es = sorted(
        float(m) for m in re.findall(r"^G1 [XY].*E([0-9.]+)", text, re.MULTILINE)
    )
    lines = text.count("\n")
    retracts = len(re.findall(r"^G1 E-", text, re.MULTILINE))
    print(f"wrote {OUT}")
    print(f"  {lines} lines, {len(text)} bytes")
    print(
        f"  per-line E: median {es[len(es) // 2]:.4f}"
        f"  p10 {es[len(es) // 10]:.4f}  p90 {es[9 * len(es) // 10]:.4f}"
        f"   (real: 0.0537 / 0.0280 / 0.0783)"
    )
    print(
        f"  retract lines: {retracts} of {lines}"
        f" = {100.0 * retracts / lines:.2f}%  (real: 0.37%)"
    )


if __name__ == "__main__":
    main()
