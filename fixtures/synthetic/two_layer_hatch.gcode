; Fixture: two fully annotated layers with a crossing sparse-infill
; hatch, relative E.
; Exercises: contact-zone selection - layer-1 (N-1) offers outer wall,
; inner wall, sparse infill and internal solid infill; layer-2 (N)
; covers the inner wall (retraced square) and the three sparse-infill
; midpoints (they lie exactly on layer-2's anti-diagonal x+y=60), but
; NOT the solid-infill line, so coverage filtering is observable.
; A crash at the hatch center (30,30) excludes the main diagonal's
; midpoint and its fallback samples, leaving the two short diagonals
; as the top (sparse-infill) candidates.
G90
M83
G92 E0
;LAYER_CHANGE
;Z:0.2
G1 Z0.2 F7200
;TYPE:Outer wall
G1 X20 Y20 F9000
G1 X40 Y20 E0.66 F1800
G1 X40 Y40 E0.66
G1 X20 Y40 E0.66
G1 X20 Y20 E0.66
;TYPE:Inner wall
G1 X22 Y22 F9000
G1 X38 Y22 E0.53 F2400
G1 X38 Y38 E0.53
G1 X22 Y38 E0.53
G1 X22 Y22 E0.53
;TYPE:Sparse infill
G1 X24 Y24 F9000
G1 X36 Y36 E0.56 F3000
G1 X24 Y30 F9000
G1 X30 Y36 E0.28 F3000
G1 X30 Y24 F9000
G1 X36 Y30 E0.28 F3000
;TYPE:Internal solid infill
G1 X24 Y25 F9000
G1 X36 Y25 E0.40 F2400
;LAYER_CHANGE
;Z:0.4
G1 E-0.8 F2100
G1 Z0.4 F7200
G1 E0.8 F2100
;TYPE:Inner wall
G1 X22 Y22 F9000
G1 X38 Y22 E0.53 F2400
G1 X38 Y38 E0.53
G1 X22 Y38 E0.53
G1 X22 Y22 E0.53
;TYPE:Sparse infill
G1 X24 Y36 F9000
G1 X36 Y24 E0.56 F3000
;TYPE:Outer wall
G1 X20 Y20 F9000
G1 X40 Y20 E0.66 F1800
G1 X40 Y40 E0.66
G1 X20 Y40 E0.66
G1 X20 Y20 E0.66
