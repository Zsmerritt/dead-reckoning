# Realistic sliced g-code fixtures

`plr-gcode`'s `real_corpus_full_pipeline` test auto-discovers every `*.gcode`
here and runs the full parse + simulate + Z-scan pipeline over each one.

## This directory was empty, and the test passed vacuously

For its entire life before `realistic_orca.gcode` landed, this directory held
only a README. The test was a bare loop over an empty list, so its body never
executed: it reported `ok` on every run while exercising nothing. Nobody
noticed, because **a vacuous test and a passing test look identical in every
report anyone reads.**

That is fixed in two places, and both matter:

* the corpus is no longer empty, and
* the test now *asserts* the corpus is non-empty, so deleting or forgetting to
  regenerate the fixture fails loudly instead of quietly restoring the vacuum.

## Why the geometry is synthetic

Real sliced files were captured from a live printer to measure what "realistic"
means (below), but the operator ruled that their own design files must not
enter this repository's permanent history — a repo is forever, and a part
someone designed is their work, not test data. So the committed fixture follows
the precedent already set by `fixtures/synthetic/*_real_footer.gcode`:

    synthetic header + synthetic body + a REAL slicer footer, verbatim

The footer is copied from the already-committed
`fixtures/synthetic/orca_real_footer.gcode`, which `footer_generator.py` had
already scrubbed and vetted, so nothing new derived from anyone's model is
added here. The `THUMBNAIL_BLOCK` a real OrcaSlicer file carries is
deliberately absent: it is a base64 PNG *rendering of the part*.

## What "realistic" was matched against

Measured over four real OrcaSlicer 2.4.2 files (0.2 and 0.4 mm layers, ABS and
ASA; 114 127 lines in the largest):

| quantity | real | `realistic_orca.gcode` |
| --- | --- | --- |
| per-line positive E, median | 0.0537 mm | 0.0454 mm |
| per-line positive E, p10 | 0.0280 mm | 0.0279 mm |
| per-line positive E, p90 | 0.0783 mm | 0.0782 mm |
| retract lines | 0.37 % | 0.41 % |

The tails are what matter: a fixture whose per-line E is a single constant has
a degenerate distribution and cannot answer questions about evidence-interval
width, which is half of why this corpus exists. The other half is that `plrd`'s
end-to-end fixture deliberately carries 0.65 mm of E on single lines to keep
stop-point matching unambiguous, and sits at exactly 8 of 8 candidates against
`ambiguity_limit` — so it has no headroom to measure anything with.

## Real files are still welcome

Drop any `*.gcode` here and it is picked up automatically. Nothing about the
suite assumes the corpus is synthetic — that is only true of what is
*committed*, and only because of the decision recorded above.

## Regenerating

    python3 fixtures/real/realistic_generator.py

Deterministic (no RNG): a diff after regeneration means a real change. The
generator prints the density table above so drift is visible.
