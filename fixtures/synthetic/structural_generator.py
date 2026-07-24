#!/usr/bin/env python3
"""Generate the structural-safety fixtures for `plr-analyzer`.

The five shapes below exercise the island/footprint/verdict machinery in
`crates/plr-analyzer/src/structure.rs`. They are generated rather than
hand-written because a solid region needs scan lines at the real 0.4 mm
line spacing to rasterize as solid material -- ~50 extrusion lines per
layer -- and hand-maintaining that is how transcription bugs get in.

Run from the repository root:

    python fixtures/synthetic/structural_generator.py

Conventions, chosen so the fixtures test *structure* and nothing else:

* Absolute XYZ (`G90`), relative E (`M83`).
* Every layer prints the identical hatch pattern, so every point on a
  layer-(N-1) centerline lies exactly on a layer-N centerline and the
  contact selector's coverage filter is always satisfied. Real slicers
  rotate the hatch between layers; that is a coverage concern, tested
  elsewhere, and rotating here would only add noise.
* `;TYPE:Internal solid infill` throughout: probe-eligible, and honest
  about what the geometry actually is.
"""

import os

LINE_SPACING = 0.4
E_PER_MM_PER_LAYER = 0.187  # 0.45 mm wide bead, 1.75 mm filament.


def extrude(x0, y0, x1, y1, layer_height):
    """One travel + one extrusion, as a list of g-code lines."""
    length = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    e = length * E_PER_MM_PER_LAYER * layer_height
    return [
        "G1 X%.3f Y%.3f F9000" % (x0, y0),
        "G1 X%.3f Y%.3f E%.4f F1800" % (x1, y1, e),
    ]


def rows(y_lo, y_hi):
    """Scan-line Y coordinates covering `y_lo..y_hi` at the line spacing."""
    out = []
    steps = int(round((y_hi - y_lo) / LINE_SPACING))
    for i in range(steps + 1):
        out.append(y_lo + i * LINE_SPACING)
    return out


def solid_rect(x_lo, x_hi, y_lo, y_hi, layer_height):
    """Scan lines filling an axis-aligned rectangle."""
    lines = []
    for y in rows(y_lo, y_hi):
        lines += extrude(x_lo, y, x_hi, y, layer_height)
    return lines


def isthmus_spans(y):
    """X spans of the isthmus fixture at scan-line `y`.

    Two 14 x 14 plates at x 0..14 and x 20..34, joined across x 14..20
    by a band of scan lines at 5.2 <= y <= 8.8 (4 mm of centerlines,
    ~4.05 mm of material).
    """
    if 5.2 - 1e-9 <= y <= 8.8 + 1e-9:
        return [(0.0, 34.0)]
    return [(0.0, 14.0), (20.0, 34.0)]


def layer_block(z, layer_height, body):
    """A complete annotated layer."""
    return [";LAYER_CHANGE", ";Z:%.2f" % z, "G1 Z%.2f F7200" % z, ";TYPE:Internal solid infill"] + body


def build(header, layers):
    out = ["; " + line for line in header]
    out += ["G90", "M83", "G92 E0"]
    for layer in layers:
        out += layer
    return "\n".join(out) + "\n"


def tall_pillar():
    header = [
        "Fixture: a 2.5 x 2.4 mm pillar, 24 layers of 0.4 mm, top at Z9.6.",
        "Exercises: every load-bearing structural criterion failing at once.",
        "The layer-0 footprint measures ~6.6 mm^2 against the 100 mm^2",
        "bed-contact bar; the aspect ratio 9.6 / sqrt(6.6) = 3.7 is over the",
        "3.0 tipping limit; the island is 2.4 mm across against the 5 mm",
        "feature-width floor; and no interior point has the 3 mm edge margin.",
        "A nozzle dragging across this shears it off the plate.",
    ]
    layers = []
    for i in range(24):
        z = 0.4 * (i + 1)
        layers.append(layer_block(z, 0.4, solid_rect(0.0, 2.5, 0.0, 2.4, 0.4)))
    return build(header, layers)


def flat_plate():
    header = [
        "Fixture: a solid 20 x 20 mm plate, 3 layers of 0.2 mm.",
        "Exercises: the all-pass case. ~344 mm^2 of traced bed contact,",
        "aspect 0.6 / 18.5 = 0.03, 20 mm across, and ~10 mm of edge margin",
        "at the centre -- every criterion clears with room to spare. Also",
        "the reference geometry for the largest-clear-run assertions: from",
        "(10,10) at a 3 mm margin the longest run is the 45-degree diagonal",
        "to the clear-region corner, ~10 mm.",
    ]
    layers = []
    for i in range(3):
        z = 0.2 * (i + 1)
        layers.append(layer_block(z, 0.2, solid_rect(0.0, 20.0, 0.0, 20.0, 0.2)))
    return build(header, layers)


def isthmus():
    header = [
        "Fixture: two solid 14 x 14 mm plates joined by a 4 mm wide, 6 mm",
        "long isthmus (x 14..20, y 5.2..8.8), 3 layers of 0.2 mm.",
        "Exercises: tap-passes / drag-fails at the SAME point. A tap at",
        "(7,7) clears every criterion -- one island, ~360 mm^2 of bed",
        "contact, 7 mm of edge margin. A 12 mm drag from (7,7) along +X has",
        "to cross the isthmus, where the material narrows to ~2 mm of",
        "half-width, so the clear run ends at x ~ 11.8 (where the distance",
        "to the re-entrant corner at (14, 9.03) falls to the 3 mm margin).",
    ]
    layers = []
    for i in range(3):
        z = 0.2 * (i + 1)
        body = []
        for y in rows(0.0, 14.0):
            for x_lo, x_hi in isthmus_spans(y):
                body += extrude(x_lo, y, x_hi, y, 0.2)
        layers.append(layer_block(z, 0.2, body))
    return build(header, layers)


def two_islands():
    header = [
        "Fixture: two solid 14 x 14 mm plates with a 4 mm gap between them",
        "(x 0..14 and x 18..32), 3 layers of 0.2 mm.",
        "Exercises: island separation and the drag run refusing to span the",
        "gap. The 4 mm gap is far wider than the 0.6 mm island link",
        "tolerance, so the two plates are distinct islands; a drag from",
        "(7,7) along +X must stop inside the first plate rather than",
        "'continue' onto the second.",
    ]
    layers = []
    for i in range(3):
        z = 0.2 * (i + 1)
        body = []
        for y in rows(0.0, 14.0):
            body += extrude(0.0, y, 14.0, y, 0.2)
            body += extrude(18.0, y, 32.0, y, 0.2)
        layers.append(layer_block(z, 0.2, body))
    return build(header, layers)


def wide_top_small_base():
    header = [
        "Fixture: a 6 x 6 mm base (layers 0-1) carrying a 20 x 20 mm flange",
        "(layers 2-3), all 0.2 mm layers, base centred under the flange.",
        "Exercises: why the footprint has to be traced down the stack. The",
        "layer-2 island measures ~344 mm^2 on its own -- comfortably over",
        "the 100 mm^2 bar -- but the material that actually holds it to the",
        "bed is the 6 x 6 base, ~31 mm^2, so bed adhesion must fail. Judging",
        "the candidate by its own layer would call this safe.",
    ]
    layers = []
    for i in range(2):
        z = 0.2 * (i + 1)
        layers.append(layer_block(z, 0.2, solid_rect(7.0, 13.0, 7.0, 13.0, 0.2)))
    for i in range(2):
        z = 0.2 * (i + 3)
        layers.append(layer_block(z, 0.2, solid_rect(0.0, 20.0, 0.0, 20.0, 0.2)))
    return build(header, layers)


FIXTURES = {
    "struct_tall_pillar.gcode": tall_pillar,
    "struct_flat_plate.gcode": flat_plate,
    "struct_isthmus.gcode": isthmus,
    "struct_two_islands.gcode": two_islands,
    "struct_wide_top_small_base.gcode": wide_top_small_base,
}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    for name, make in sorted(FIXTURES.items()):
        path = os.path.join(here, name)
        with open(path, "w", newline="\n") as handle:
            handle.write(make())
        print("wrote %s" % name)


if __name__ == "__main__":
    main()
