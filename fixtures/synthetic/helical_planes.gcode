; Fixture: helical arcs and non-default arc planes.
; Exercises: G17 helical arc (Z ramps across chords -> extruding
; Z events), G18 XZ-plane arc with I/K words (Y helical), G19
; YZ-plane arc with J/K words (X helical), then return to G17.
G90
M82
G92 E0
G1 X10 Y0 Z0.4 F6000
; helical quarter circle: Z rises 0.4 -> 1.2 across the arc
G3 X0 Y10 Z1.2 I-10 E2 F1800
G17
G1 X10 Y10 Z5 F6000
; XZ plane: half circle below, through (15,10,0) back up to (20,10,5)
G18
G2 X20 Z5 I5 K0 F3600
; YZ plane: quarter circle (clockwise keeps Z positive)
G19
G2 Y20 Z15 J10 K0 F3600
G17
G1 X0 Y0 Z15 F6000
