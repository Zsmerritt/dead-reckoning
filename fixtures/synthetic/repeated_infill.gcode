; Fixture: repeated identical sparse-infill traces, relative E.
; Exercises: the matcher's ambiguity policy - three extrusions (and
; the travels between them) retrace the byte-identical XY segment, so
; XY/Z evidence alone must yield AmbiguousWindow (or LayerOnly at a
; low ambiguity limit), never a fake unique line; only a tight
; internal-E interval may disambiguate, because relative E makes each
; retrace land on a different cumulative E.
G90
M83
G92 E0
G1 Z0.2 F7200
;TYPE:Sparse infill
G1 X20 Y20 F9000
G1 X40 Y20 E0.5 F3000
G1 X20 Y20 F9000
G1 X40 Y20 E0.5 F3000
G1 X20 Y20 F9000
G1 X40 Y20 E0.5 F3000
