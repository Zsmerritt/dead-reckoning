; generated-by dead-reckoning power-loss recovery
; generated-at TS
; source-file part.gcode
; matched-offset 128
; plan-id plr-128
; --- original file header (metadata) ---
; --- end original file header ---
M140 S60
M104 S210
M190 S60
M109 S210
G28 X Y
G92 E0
G1 E5 F300
G92 E0
G90
M83
G0 Z1.35 F1200
G0 X30 Y30 F1200
G1 Z0.35 F1200
G1 E0.4 F1800
G92 E3
M83
G90
G1 F1800
G1 X10 Y10 E1 F1800
G1 X30 Y10 E1
G1 X30 Y30 E1
