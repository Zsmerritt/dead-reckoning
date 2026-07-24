; Fixture: one extruded square, relative E (M83).
; Exercises: relative-E accounting; equivalence partner of
; abs_e_square.gcode - the fixture test asserts both end at the same
; internal positions after the same toolpath.
G90
M83
G92 E0
G1 Z0.2 F7200
G1 X10 Y10 F9000
G1 X50 Y10 E1.5 F1800
G1 X50 Y50 E1.5
G1 X10 Y50 E1.5
G1 X10 Y10 E1.5
G1 E-0.8 F2100
