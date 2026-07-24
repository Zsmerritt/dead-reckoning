; Fixture: one extruded square, absolute E (M82).
; Exercises: absolute-E accounting; equivalence partner of
; rel_e_square.gcode - the fixture test asserts both end at the same
; internal positions after the same toolpath.
G90
M82
G92 E0
G1 Z0.2 F7200
G1 X10 Y10 F9000
G1 X50 Y10 E1.5 F1800
G1 X50 Y50 E3.0
G1 X10 Y50 E4.5
G1 X10 Y10 E6.0
G1 E5.2 F2100
