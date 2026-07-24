; Fixture: vase mode (spiral Z), PrusaSlicer style.
; Exercises: continuous Z ramp during extrusion (every move is a
; Z-touching, extruding move), the Z-event scan's spiral flagging,
; and absolute-E bookkeeping over many small moves.
G90
M82
G92 E0
G1 Z0.2 F7200
G1 X50 Y30 F9000
;TYPE:External perimeter
G1 X70 Y30 Z0.212 E0.65 F1500
G1 X70 Y50 Z0.225 E1.30
G1 X50 Y50 Z0.237 E1.95
G1 X50 Y30 Z0.250 E2.60
G1 X70 Y30 Z0.262 E3.25
G1 X70 Y50 Z0.275 E3.90
G1 X50 Y50 Z0.287 E4.55
G1 X50 Y30 Z0.300 E5.20
G1 X70 Y30 Z0.312 E5.85
G1 X70 Y50 Z0.325 E6.50
G1 X50 Y50 Z0.337 E7.15
G1 X50 Y30 Z0.350 E7.80
G1 X70 Y30 Z0.362 E8.45
G1 X70 Y50 Z0.375 E9.10
G1 X50 Y50 Z0.387 E9.75
G1 X50 Y30 Z0.400 E10.40
