; Fixture: dedicated z-hop-on-retract dance, relative E.
; Exercises: the exact Z-event sequence the Z scan must reproduce -
; hop up 0.4, travel, hop down 0.4, repeated, plus a layer change;
; retracts (negative E) between hops.
G90
M83
G92 E0
G1 Z0.2 F7200
G1 X10 Y10 F9000
G1 X40 Y10 E1.0 F1800
G1 E-0.8 F2100
G1 Z0.6 F7200
G1 X80 Y10 F9000
G1 Z0.2 F7200
G1 E0.8 F2100
G1 X80 Y40 E1.0 F1800
G1 E-0.8 F2100
G1 Z0.6 F7200
G1 X10 Y40 F9000
G1 Z0.2 F7200
G1 E0.8 F2100
G1 X10 Y10 E1.0 F1800
;LAYER_CHANGE
G1 E-0.8 F2100
G1 Z0.4 F7200
G1 E0.8 F2100
G1 X40 Y40 E1.2 F1800
