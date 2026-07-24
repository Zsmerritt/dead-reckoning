; Fixture: G28 in the middle of a file (e.g. a probing macro's output).
; Exercises: position-knowledge tracking - after G28 the homed axes
; are unknown until re-established by absolute moves; the Z scan flags
; Z events with z_known=false in the unknown window.
G90
M82
G92 E0
G1 Z0.2 F7200
G1 X10 Y10 F9000
G1 X30 Y10 E1.0 F1800
G28 Z
G91
G1 Z2 F7200
G90
G1 Z0.4 F7200
G1 X30 Y30 E2.0 F1800
G28
G1 X10 Y10 Z0.6 F6000
G1 X30 Y10 E3.0 F1800
