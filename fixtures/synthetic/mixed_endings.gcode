; Fixture: CRLF line endings, N line-numbers with checksums,
; lowercase words, blank lines, and inline comments.
; Exercises: terminator handling, byte-span tiling under CRLF,
; line-number/checksum stripping, case folding.
N1 G90*37
N2 M82*29
N3 G92 E0*102

n4 g1 z0.2 f7200*88
N5 G1 X10 Y10 F9000*87
N6 G1 X30 Y10 E1.0 F1800*90
   
N7 G1 E-0.8 F2100*110 ; retract
N8 G1 Z0.6*44
N9 G1 X60 Y10*61
N10 G1 Z0.2*23
N11 G1 E0.8*57
N12 G1 X60 Y40 E2.0*12
