; Fixture: G2/G3 arcs, IJ form, XY plane (G17), absolute E.
; Exercises: CCW quarter circle with E+F, CW full circle (target equals
; current position), travel arc without E, retract after arcs.
; Equivalence partner: arcs_prechorded.gcode contains the same toolpath
; with every arc pre-decomposed at resolution 1.0 (arcs "off"); the
; fixture test asserts both produce the same move endpoints.
G90
M82
G92 E0
G1 X10 Y0 Z0.4 F6000
G3 X0 Y10 I-10 E3 F1800
G1 X20 Y10 F6000
G2 I-10 E9 F1800
G3 X30 Y10 I5 F3600
G1 E8.5 F2100
