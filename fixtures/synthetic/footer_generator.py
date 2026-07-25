#!/usr/bin/env python3
"""Regenerates the footer-bearing fixtures.

`fixtures/synthetic/` had no fixture with a *footer* -- the end g-code plus
the serialized print profile every mainstream slicer appends after the last
extruding move.  That absence is exactly why nothing in the test suite
could see that a functionally complete print stops well short of EOF, and
therefore why a percentage-based "~95% complete" announcement survived
every gate.

Four fixtures are produced.

## The two `*_real_footer` fixtures (preferred; used by the goldens)

    prusa_real_footer.gcode   PrusaSlicer 2.9.3
    orca_real_footer.gcode    OrcaSlicer 2.3.1

Each is **a synthetic header + a synthetic body + a real footer verbatim**.

The footers were extracted from genuine prints sliced by the named slicer
versions, taking the last positive-extrusion line through EOF.  Only the
footer is real, and that is deliberate:

* it is the only part the completion gate reasons about, and
* it is slicer-generated boilerplate -- end g-code, wipe trail, and the
  serialized print profile -- rather than model geometry.

The models those prints were of are third-party and not redistributable
(the one used here is licensed CC BY-ND) and this repository is public, so
**no whole real file is committed and no real toolpath is committed**.  Exactly two things
are substituted, and nothing else is altered:

1. **The model file name**, everywhere it appears -- including inside
   PrusaSlicer's `; objects_info = {...}` line and Orca's
   `EXCLUDE_OBJECT_END NAME=`.
2. **The model footprint outline.**  PrusaSlicer's `; objects_info` line
   carries a 52-point 2D outline of the printed object -- 899 bytes of
   coordinates.  That was the only committed content that was geometry *of
   the model* rather than slicer boilerplate, so it is replaced with an
   obviously-synthetic placeholder (`PLACEHOLDER_POLYGON`: a 20x20 mm
   square with 5 mm-spaced collinear vertices) that keeps the line's key
   names, nesting and numeric formatting intact.  It costs the tests
   nothing -- `objects_info` is a comment, the parser treats it as one
   whether it holds 52 points or none, and the completion gate never reads
   it -- and it removes the only arguable licensing question.

Every other byte of both footers is verbatim, so the fixtures keep the
exact lexical forms real slicers emit.  `tests/fixtures.rs` asserts that
neither the original model name nor the original outline can reappear.

Measured footer sizes -- state these, not estimates.  "As written" is
what the slicer actually emitted (the extraction input); "committed" is
after the two substitutions above, which is what the fixture holds:

| slicer            | as written | committed | footer lines |
|-------------------|------------|-----------|--------------|
| PrusaSlicer 2.9.3 | 14,537 B   | 13,913 B  | 403          |
| OrcaSlicer 2.3.1  | 17,699 B   | 17,695 B  | 560          |

The PrusaSlicer figure loses 624 bytes to the outline placeholder (899
bytes of real coordinates replaced by 275) and the name; Orca loses only
the name.  Either way the argument is unchanged: a finished print's last
depositing line sits **14 KB (Prusa) to 18 KB (Orca)** before EOF, and
that distance barely moves with print size.

## `cura_footer_complete.gcode` (fully synthetic)

A third dialect, and the only fixture whose **footer contains extrusion-mode
commands**: Cura's stock end g-code switches to relative positioning for the
wipe-out move and back again (`G91` ... `G90`).  That exercises
`plr_analyzer::work`'s extruder-frame trust check on real-shaped input --
without it, no fixture in the corpus reaches the branch, and an
over-refusal on ordinary Cura output would go unnoticed.

## The two `*_footer_complete` fixtures (fully synthetic)

    prusa_footer_complete.gcode
    orca_footer_complete.gcode

Kept because they are fully synthetic and therefore freely editable, and
because they bracket *deposition* with
`EXCLUDE_OBJECT_DEFINE`/`START`/`END`, which the real-footer fixtures can
only do minimally (their real deposition is not committed).  Their config
blocks are modelled on a real 324-key block but come out at 12,522 bytes --
about 0.86x the real PrusaSlicer footer and 0.71x the real Orca one, i.e.
the right order of magnitude but low.  Prefer the real-footer fixtures for
any measurement.

Thumbnails are deliberately absent from all four: slicers write them in the
*header* (`; thumbnail begin`), not the footer, so they do not contribute to
the end-of-file distance the completion gate has to reason about.

Run from anywhere; writes next to this script.  The raw footers live outside
the repository (they are extraction inputs, not artifacts); point
`--footer-dir` at them to regenerate the real-footer pair, or run with no
arguments to regenerate only the synthetic pair.

    python3 fixtures/synthetic/footer_generator.py [--footer-dir DIR]

## Not covered by the lint gates

`ruff` (both `check` and `format`) and the python coverage gate run over
`klippy_plugin/` only, so nothing in `fixtures/` is linted, formatted or
tested by the pre-commit hook or CI -- this file included.  It is a
developer tool that produces committed artifacts, and the artifacts are what
the Rust goldens assert against, so a mistake here shows up as a failing
Rust test rather than as a lint error.  Bringing `fixtures/` into ruff's
scope would be a `scripts/` change; until someone does, treat this file as
hand-checked.
"""

from __future__ import annotations

import argparse
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))

# What the model file name is replaced with.
PLACEHOLDER_NAME = "part_a.stl"

# The real name is *derived from the input*, never written down here: this
# file is committed, and hardcoding the name would put it in the repository
# and its history — the very thing the substitution exists to avoid. Every
# slicer labels object boundaries with this line, which is where the name is
# read from:
#
#     ; stop printing object <name> id:0 copy 0
#
# Orca's `EXCLUDE_OBJECT_END NAME=<name>_id_0_copy_0` and PrusaSlicer's
# `; objects_info = {"objects":[{"name":"<name> id:0 copy 0", ...`  both
# embed the same string, so replacing it once covers them.
STOP_PRINTING_OBJECT = re.compile(r"^; stop printing object (?P<name>.+?) id:", re.M)

# A representative slice of a real PrusaSlicer config block: the key
# spelling and the ``; key = value`` layout are what matter, so the list
# is padded out to the real key count with plausible per-extruder and
# per-filament keys rather than invented nonsense.
PRUSA_KEYS = [
    ("avoid_crossing_perimeters", "0"),
    ("bed_shape", "0x0,250x0,250x210,0x210"),
    ("bed_temperature", "60"),
    ("before_layer_gcode", ";BEFORE_LAYER_CHANGE\\nG92 E0.0\\n;[layer_z]\\n\\n"),
    ("between_objects_gcode", ""),
    ("bottom_fill_pattern", "monotonic"),
    ("bottom_solid_layers", "4"),
    ("bottom_solid_min_thickness", "0.5"),
    ("bridge_acceleration", "1000"),
    ("bridge_angle", "0"),
    ("bridge_fan_speed", "100"),
    ("bridge_flow_ratio", "1"),
    ("bridge_speed", "25"),
    ("brim_separation", "0"),
    ("brim_type", "outer_only"),
    ("brim_width", "0"),
    ("clip_multipart_objects", "1"),
    ("colorprint_heights", ""),
    ("complete_objects", "0"),
    ("cooling", "1"),
    ("cooling_tube_length", "5"),
    ("cooling_tube_retraction", "91.5"),
    ("default_acceleration", "1000"),
    ("default_filament_profile", "Prusament PLA"),
    ("default_print_profile", "0.20mm QUALITY @MK3"),
    ("deretract_speed", "0"),
    ("disable_fan_first_layers", "1"),
    ("dont_support_bridges", "1"),
    ("draft_shield", "disabled"),
    ("duplicate_distance", "6"),
    ("elefant_foot_compensation", "0.2"),
    ("end_filament_gcode", '"; Filament-specific end gcode"'),
    ("external_perimeter_extrusion_width", "0.45"),
    ("external_perimeter_speed", "25"),
    ("external_perimeters_first", "0"),
    ("extra_loading_move", "-2"),
    ("extra_perimeters", "0"),
    ("extruder_clearance_height", "20"),
    ("extruder_clearance_radius", "45"),
    ("extruder_colour", ""),
    ("extruder_offset", "0x0"),
    ("extrusion_axis", "E"),
    ("extrusion_multiplier", "1"),
    ("extrusion_width", "0.45"),
    ("fan_always_on", "1"),
    ("fan_below_layer_time", "100"),
    ("filament_colour", "#FF8000"),
    ("filament_cooling_final_speed", "3.4"),
    ("filament_cooling_initial_speed", "2.2"),
    ("filament_cooling_moves", "4"),
    ("filament_cost", "24.99"),
    ("filament_density", "1.24"),
    ("filament_deretract_speed", "nil"),
    ("filament_diameter", "1.75"),
    ("filament_load_time", "0"),
    ("filament_loading_speed", "28"),
    ("filament_loading_speed_start", "3"),
    ("filament_max_volumetric_speed", "15"),
    ("filament_minimal_purge_on_wipe_tower", "15"),
    ("filament_notes", '""'),
    ("filament_ramming_parameters", '"120 100 6.6 6.8 7.2 7.6 7.9 8.2 8.7 9.4 9.9"'),
    ("filament_retract_before_travel", "nil"),
    ("filament_retract_before_wipe", "nil"),
    ("filament_retract_layer_change", "nil"),
    ("filament_retract_length", "nil"),
    ("filament_retract_lift", "nil"),
    ("filament_retract_restart_extra", "nil"),
    ("filament_retract_speed", "nil"),
    ("filament_settings_id", '"Prusament PLA"'),
    ("filament_soluble", "0"),
    ("filament_spool_weight", "201"),
    ("filament_toolchange_delay", "0"),
    ("filament_type", "PLA"),
    ("filament_unload_time", "0"),
    ("filament_unloading_speed", "90"),
    ("filament_unloading_speed_start", "100"),
    ("filament_vendor", "Prusa Polymers"),
    ("fill_angle", "45"),
    ("fill_density", "15%"),
    ("fill_pattern", "gyroid"),
    ("first_layer_acceleration", "800"),
    ("first_layer_acceleration_over_raft", "0"),
    ("first_layer_bed_temperature", "60"),
    ("first_layer_extrusion_width", "0.42"),
    ("first_layer_height", "0.2"),
    ("first_layer_speed", "20"),
    ("first_layer_speed_over_raft", "30"),
    ("first_layer_temperature", "215"),
    ("full_fan_speed_layer", "4"),
    ("fuzzy_skin", "none"),
    ("fuzzy_skin_point_dist", "0.8"),
    ("fuzzy_skin_thickness", "0.3"),
    ("gap_fill_enabled", "1"),
    ("gap_fill_speed", "40"),
    ("gcode_comments", "0"),
    ("gcode_flavor", "marlin2"),
    ("gcode_label_objects", "1"),
    ("gcode_resolution", "0.0125"),
    ("gcode_substitutions", ""),
    ("high_current_on_filament_swap", "0"),
    ("host_type", "octoprint"),
    ("infill_acceleration", "1000"),
    ("infill_anchor", "2.5"),
    ("infill_anchor_max", "12"),
    ("infill_every_layers", "1"),
    ("infill_extruder", "1"),
    ("infill_extrusion_width", "0.45"),
    ("infill_first", "0"),
    ("infill_only_where_needed", "0"),
    ("infill_overlap", "25%"),
    ("infill_speed", "200"),
    ("interface_shells", "0"),
    ("ironing", "0"),
    ("ironing_flowrate", "15%"),
    ("ironing_spacing", "0.1"),
    ("ironing_speed", "15"),
    ("ironing_type", "top"),
    ("layer_gcode", ";AFTER_LAYER_CHANGE\\n;[layer_z]"),
    ("layer_height", "0.2"),
    ("machine_limits_usage", "emit_to_gcode"),
    ("machine_max_acceleration_e", "5000,5000"),
    ("machine_max_acceleration_extruding", "1250,1250"),
    ("machine_max_acceleration_retracting", "1250,1250"),
    ("machine_max_acceleration_travel", "1500,1250"),
    ("machine_max_acceleration_x", "1000,960"),
    ("machine_max_acceleration_y", "1000,960"),
    ("machine_max_acceleration_z", "1000,1000"),
    ("machine_max_feedrate_e", "120,120"),
    ("machine_max_feedrate_x", "200,100"),
    ("machine_max_feedrate_y", "200,100"),
    ("machine_max_feedrate_z", "12,12"),
    ("machine_max_jerk_e", "4.5,4.5"),
    ("machine_max_jerk_x", "8,8"),
    ("machine_max_jerk_y", "8,8"),
    ("machine_max_jerk_z", "0.4,0.4"),
    ("machine_min_extruding_rate", "0,0"),
    ("machine_min_travel_rate", "0,0"),
    ("max_fan_speed", "100"),
    ("max_layer_height", "0.25"),
    ("max_print_height", "210"),
    ("max_print_speed", "200"),
    ("max_volumetric_extrusion_rate_slope_negative", "0"),
    ("max_volumetric_extrusion_rate_slope_positive", "0"),
    ("max_volumetric_speed", "0"),
    ("min_bead_width", "85%"),
    ("min_fan_speed", "100"),
    ("min_feature_size", "25%"),
    ("min_layer_height", "0.07"),
    ("min_print_speed", "15"),
    ("min_skirt_length", "4"),
    ("mmu_segmented_region_max_width", "0"),
    ("notes", ""),
    ("nozzle_diameter", "0.4"),
    ("nozzle_high_flow", "0"),
    ("only_retract_when_crossing_perimeters", "0"),
    ("ooze_prevention", "0"),
    ("output_filename_format", "{input_filename_base}_{layer_height}mm.gcode"),
    ("overhangs", "1"),
    ("parking_pos_retraction", "92"),
    ("pause_print_gcode", "M601"),
    ("perimeter_acceleration", "800"),
    ("perimeter_extruder", "1"),
    ("perimeter_extrusion_width", "0.45"),
    ("perimeter_generator", "arachne"),
    ("perimeter_speed", "45"),
    ("perimeters", "2"),
    ("physical_printer_settings_id", ""),
    ("post_process", ""),
    ("print_settings_id", "0.20mm QUALITY @MK3"),
    ("printer_model", "MK3S"),
    ("printer_notes", "Don't remove the following keywords!"),
    ("printer_settings_id", "Original Prusa i3 MK3S & MK3S+"),
    ("printer_technology", "FFF"),
    ("printer_variant", "0.4"),
    ("printer_vendor", ""),
    ("raft_contact_distance", "0.1"),
    ("raft_expansion", "1.5"),
    ("raft_first_layer_density", "90%"),
    ("raft_first_layer_expansion", "3"),
    ("raft_layers", "0"),
    ("remaining_times", "1"),
    ("resolution", "0"),
    ("retract_before_travel", "1"),
    ("retract_before_wipe", "70%"),
    ("retract_layer_change", "1"),
    ("retract_length", "0.8"),
    ("retract_length_toolchange", "4"),
    ("retract_lift", "0.4"),
    ("retract_lift_above", "0"),
    ("retract_lift_below", "209"),
    ("retract_restart_extra", "0"),
    ("retract_restart_extra_toolchange", "0"),
    ("retract_speed", "35"),
    ("seam_position", "aligned"),
    ("silent_mode", "1"),
    ("single_extruder_multi_material", "0"),
    ("single_extruder_multi_material_priming", "0"),
    ("skirt_distance", "2"),
    ("skirt_height", "3"),
    ("skirts", "0"),
    ("slice_closing_radius", "0.049"),
    ("slicing_mode", "regular"),
    ("slowdown_below_layer_time", "20"),
    ("small_perimeter_speed", "25"),
    ("solid_infill_below_area", "0"),
    ("solid_infill_every_layers", "0"),
    ("solid_infill_extruder", "1"),
    ("solid_infill_extrusion_width", "0.45"),
    ("solid_infill_speed", "200"),
    ("spiral_vase", "0"),
    ("staggered_inner_seams", "0"),
    ("standby_temperature_delta", "-5"),
    ("start_filament_gcode", '"M900 K0.05 ; Filament gcode LA 1.5"'),
    ("support_material", "0"),
    ("support_material_angle", "0"),
    ("support_material_auto", "1"),
    ("support_material_bottom_contact_distance", "0"),
    ("support_material_bottom_interface_layers", "-1"),
    ("support_material_buildplate_only", "0"),
    ("support_material_closing_radius", "2"),
    ("support_material_contact_distance", "0.1"),
    ("support_material_enforce_layers", "0"),
    ("support_material_extruder", "0"),
    ("support_material_extrusion_width", "0.35"),
    ("support_material_interface_contact_loops", "0"),
    ("support_material_interface_extruder", "0"),
    ("support_material_interface_layers", "2"),
    ("support_material_interface_pattern", "rectilinear"),
    ("support_material_interface_spacing", "0.2"),
    ("support_material_interface_speed", "80%"),
    ("support_material_pattern", "rectilinear"),
    ("support_material_spacing", "2"),
    ("support_material_speed", "50"),
    ("support_material_style", "snug"),
    ("support_material_synchronize_layers", "0"),
    ("support_material_threshold", "55"),
    ("support_material_with_sheath", "0"),
    ("support_material_xy_spacing", "50%"),
    ("temperature", "210"),
    ("template_custom_gcode", ""),
    ("thick_bridges", "0"),
    ("thin_walls", "0"),
    ("threads", "12"),
    ("thumbnails", "16x16/QOI, 313x173/QOI"),
    ("toolchange_gcode", ""),
    ("top_fill_pattern", "monotoniclines"),
    ("top_infill_extrusion_width", "0.4"),
    ("top_solid_infill_acceleration", "1000"),
    ("top_solid_infill_speed", "40"),
    ("top_solid_layers", "5"),
    ("top_solid_min_thickness", "0.7"),
    ("travel_acceleration", "1250"),
    ("travel_speed", "180"),
    ("travel_speed_z", "12"),
    ("use_firmware_retraction", "0"),
    ("use_relative_e_distances", "0"),
    ("use_volumetric_e", "0"),
    ("variable_layer_height", "1"),
    ("wall_distribution_count", "1"),
    ("wall_transition_angle", "10"),
    ("wall_transition_filter_deviation", "25%"),
    ("wall_transition_length", "100%"),
    ("wipe", "1"),
    ("wipe_into_infill", "0"),
    ("wipe_into_objects", "0"),
    ("wipe_tower", "1"),
    ("wipe_tower_bridging", "10"),
    ("wipe_tower_brim_width", "2"),
    ("wipe_tower_cone_angle", "0"),
    ("wipe_tower_extra_spacing", "100%"),
    ("wipe_tower_no_sparse_layers", "0"),
    ("wipe_tower_rotation_angle", "0"),
    ("wipe_tower_width", "60"),
    ("wipe_tower_x", "170"),
    ("wipe_tower_y", "125"),
    ("wiping_volumes_extruders", "70,70"),
    ("wiping_volumes_matrix", "0"),
    ("xy_size_compensation", "0"),
    ("z_offset", "0"),
]

# The real block is 324 keys and ~14 KB, i.e. ~43 bytes per line: the
# long tail is per-extruder / per-filament override keys whose values are
# quoted gcode snippets, not bare integers. Pad in that shape so the
# fixture's end-of-file distance is the right order of magnitude.
PAD_TEMPLATE = "filament_custom_gcode_override_{:02d}"
PAD_VALUE = '"M900 K0.05 ; per-extruder linear-advance override"'
TARGET_KEYS = 324


def padded_keys() -> "list[tuple[str, str]]":
    keys = list(PRUSA_KEYS)
    i = 0
    while len(keys) < TARGET_KEYS:
        keys.append((PAD_TEMPLATE.format(i), PAD_VALUE))
        i += 1
    return keys[:TARGET_KEYS]


def prusa_config_block() -> "list[str]":
    keys = padded_keys()
    lines = ["; prusaslicer_config = begin"]
    lines += ["; {} = {}".format(k, v) for k, v in keys]
    lines.append("; prusaslicer_config = end")
    return lines


def orca_config_block() -> "list[str]":
    # Orca serializes the same information between explicit block markers.
    #
    # `use_relative_e_distances` MUST agree with the body's `M83`. A config
    # block that contradicts its own file is the exact inconsistency the
    # real-footer fixtures were fixed for; a synthetic fixture has no excuse
    # for it, and a reader who trusts the block over the body would replay
    # the whole file in the wrong extruder frame.
    keys = [
        ("use_relative_e_distances", "1") if k == "use_relative_e_distances" else (k, v)
        for k, v in padded_keys()
    ]
    lines = ["; CONFIG_BLOCK_START"]
    lines += ["; {} = {}".format(k, v) for k, v in keys]
    lines.append("; CONFIG_BLOCK_END")
    return lines


# The marker line the goldens locate. It sits immediately after the last
# positive-extrusion line of the body, so "the whole footer" is exactly
# "everything at or after this comment".
LAST_DEPOSITION_MARKER = "; THE LAST DEPOSITING LINE IS ABOVE THIS COMMENT"

PRUSA_BODY = """\
; Fixture: PrusaSlicer-style file WITH a synthetic footer (end gcode +
; config block). Fully synthetic and therefore freely editable; prefer
; prusa_real_footer.gcode for any measurement.
;
; Exercises: the completion gate, plus EXCLUDE_OBJECT brackets around
; deposition (two objects, two layers) for the excluded-work path.
; generated by PrusaSlicer 2.8.1+win64 on 2026-07-01 at 12:00:00 UTC
;
; external perimeters extrusion width = 0.45mm
M82 ; absolute E
G90
M104 S215
M140 S60
M109 S215
M190 S60
G28 ; home all
G92 E0
EXCLUDE_OBJECT_DEFINE NAME=part_a CENTER=40,40 POLYGON=[[30,30],[50,30],[50,50],[30,50]]
EXCLUDE_OBJECT_DEFINE NAME=part_b CENTER=80,40 POLYGON=[[70,30],[90,30],[90,50],[70,50]]
;LAYER_CHANGE
;Z:0.2
G1 Z0.2 F7200
EXCLUDE_OBJECT_START NAME=part_a
;TYPE:External perimeter
G1 X30 Y30 F9000
G1 X50 Y30 E0.6221 F1500
G1 X50 Y50 E1.2442
G1 X30 Y50 E1.8663
G1 X30 Y30 E2.4884
;TYPE:Solid infill
G1 X32 Y32 F9000
G1 X48 Y48 E3.1884 F2400
EXCLUDE_OBJECT_END
EXCLUDE_OBJECT_START NAME=part_b
;TYPE:External perimeter
G1 E2.3884 F2100 ; retract
G1 Z0.6 F7200
G1 X70 Y30 F9000
G1 Z0.2 F7200
G1 E3.1884 F2100
G1 X90 Y30 E3.8105 F1500
G1 X90 Y50 E4.4326
G1 X70 Y50 E5.0547
G1 X70 Y30 E5.6768
EXCLUDE_OBJECT_END
;LAYER_CHANGE
;Z:0.4
G92 E0
G1 Z0.4 F7200
EXCLUDE_OBJECT_START NAME=part_a
;TYPE:External perimeter
G1 X30 Y30 F9000
G1 X50 Y30 E0.6221 F1500
G1 X50 Y50 E1.2442
G1 X30 Y50 E1.8663
G1 X30 Y30 E2.4884
EXCLUDE_OBJECT_END
EXCLUDE_OBJECT_START NAME=part_b
G1 X70 Y30 F9000
G1 X90 Y30 E3.1105 F1500
G1 X90 Y50 E3.7326
G1 X70 Y50 E4.3547
G1 X70 Y30 E4.9768
EXCLUDE_OBJECT_END
"""

PRUSA_SYNTHETIC_FOOTER = """\
; --- end gcode ---
M107 ; fan off
M104 S0 ; nozzle cooldown
M140 S0 ; bed cooldown
G1 E-0.8 F2100 ; final retract
G4 P100
M400
G1 Z10.4 F720 ; lift Z
G1 X0 Y200 F4800 ; park
M84 ; motors off
M117 Print finished
"""

ORCA_BODY = """\
; Fixture: OrcaSlicer-style file WITH a synthetic footer. Fully synthetic
; and therefore freely editable; prefer orca_real_footer.gcode for any
; measurement.
;
; Exercises: the completion gate against Orca's dialect -- relative E,
; Orca feature names, EXECUTABLE_BLOCK_END, and the config block between
; ; CONFIG_BLOCK_START and ; CONFIG_BLOCK_END.
; HEADER_BLOCK_START
; generated by OrcaSlicer 2.1.1 on 2026-07-01 at 12:00:00
; total layer number: 2
; HEADER_BLOCK_END
; EXECUTABLE_BLOCK_START
M83 ; relative E
G90
M104 S220
M140 S60
M109 S220
M190 S60
G28
M204 S500
EXCLUDE_OBJECT_DEFINE NAME=Body1_id_0_copy_0 CENTER=45,45 POLYGON=[[35,35],[55,35],[55,55],[35,55]]
;LAYER_CHANGE
; layer num/total_layer_count: 1/2
G1 Z0.2 F18000
EXCLUDE_OBJECT_START NAME=Body1_id_0_copy_0
;TYPE:Outer wall
G1 X35 Y35 F18000
G1 X55 Y35 E0.7465 F2100
G1 X55 Y55 E0.7465
G1 X35 Y55 E0.7465
G1 X35 Y35 E0.7465
;TYPE:Internal solid infill
G1 X37 Y37 F18000
G1 X53 Y53 E0.8300 F4800
EXCLUDE_OBJECT_END
;LAYER_CHANGE
; layer num/total_layer_count: 2/2
G1 E-0.8 F1800
G1 Z0.6 F18000
G1 X35 Y35 F18000
G1 Z0.4 F18000
G1 E0.8 F1800
EXCLUDE_OBJECT_START NAME=Body1_id_0_copy_0
;TYPE:Outer wall
G1 X55 Y35 E0.7465 F2100
G1 X55 Y55 E0.7465
EXCLUDE_OBJECT_END
"""

ORCA_SYNTHETIC_FOOTER = """\
;TYPE:Custom
; filament end gcode
M107
M104 S0
M140 S0
G1 E-0.8 F1800
G1 Z10.4 F600
G1 X0 Y220 F18000
M84
; EXECUTABLE_BLOCK_END
"""

# Bodies for the real-footer fixtures. These are deliberately SHORT and
# fully synthetic. Each one ends on a positive-extrusion line written with
# a LEADING-DOT float (`E.03577`) -- the form both PrusaSlicer and
# OrcaSlicer actually emit, which no other fixture in the corpus contains --
# and its XY lands on the wipe trail's start point so the real footer's
# first wipe move is geometrically continuous with the body.
PRUSA_REAL_BODY = """\
; Fixture: synthetic body + a REAL PrusaSlicer 2.9.3 footer (verbatim).
; See fixtures/synthetic/footer_generator.py for why only the footer is
; real and how the model name was scrubbed.
;
; Exercises: the completion gate against genuine slicer output --
;   * a 403-line footer -- 14,537 bytes as PrusaSlicer wrote it, 13,913 as
;     committed (see footer_generator.py) -- so the last depositing line
;     sits that far from EOF, which no percentage threshold can see;
;   * leading-dot floats (`E-.14`, `E-.01429`) in the retract and wipe;
;   * a WIPE trail: real XY motion carrying NEGATIVE E, which is why the
;     gate tests for remaining *positive extrusion* and not for remaining
;     *motion*;
;   * `; stop printing object ...` in its unconverted comment form.
;
; RELATIVE E (M83), matching the committed config block's
; `use_relative_e_distances = 1`. That is not cosmetic: the footer's
; `G1 E-.14` retract and its `E-.01429` wipe values are relative deltas,
; and replaying them in absolute mode would turn the wipe into a POSITIVE
; E move -- i.e. would make the gate see deposition in the footer.
; generated by PrusaSlicer 2.9.3+win64 on 2026-07-20 at 09:00:00 UTC
M83 ; relative E
G90
M104 S215
M140 S60
M109 S215
M190 S60
G28 ; home all
G92 E0
;LAYER_CHANGE
;Z:0.2
G1 Z0.2 F7200
;TYPE:External perimeter
G1 X95 Y95 F9000
G1 X155 Y95 E1.8663 F1500
G1 X155 Y120 E.7777
G1 X95 Y120 E1.8663
G1 X95 Y95 E.7777
;TYPE:Solid infill
G1 X100 Y100 F9000
G1 X117.121 Y105.942 E.03577
"""

ORCA_REAL_BODY = """\
; Fixture: synthetic body + a REAL OrcaSlicer 2.3.1 footer (verbatim).
; See fixtures/synthetic/footer_generator.py for why only the footer is
; real and how the model name was scrubbed.
;
; Exercises: the completion gate against genuine slicer output --
;   * a 17,695-byte / 560-line footer, the largest in the corpus
;     (17,699 as OrcaSlicer wrote it; only the model name was replaced);
;   * leading-dot floats (`E-.64987`, `E-.03364`) in the retract and wipe,
;     including a POSITIVE one (`E.03577`) on the last depositing line;
;   * a WIPE trail: real XY motion carrying NEGATIVE E;
;   * `EXCLUDE_OBJECT_END NAME=...` in its real command form, emitted
;     inside the footer.
;
; RELATIVE E (M83), matching the committed config block's
; `use_relative_e_distances = 1` -- see prusa_real_footer.gcode for why
; that agreement is load-bearing and not cosmetic.
; HEADER_BLOCK_START
; generated by OrcaSlicer 2.3.1 on 2026-07-20 at 09:00:00
; HEADER_BLOCK_END
; EXECUTABLE_BLOCK_START
M83 ; relative E
G90
M104 S220
M140 S60
M109 S220
M190 S60
G28
EXCLUDE_OBJECT_DEFINE NAME=part_a.stl_id_0_copy_0 CENTER=110,110 POLYGON=[[100,100],[120,100],[120,120],[100,120]]
;LAYER_CHANGE
; layer num/total_layer_count: 1/1
G1 Z0.2 F18000
EXCLUDE_OBJECT_START NAME=part_a.stl_id_0_copy_0
;TYPE:Outer wall
G1 X100 Y100 F18000
G1 X120 Y100 E.7465 F2100
G1 X120 Y120 E.7465
G1 X100 Y120 E.7465
G1 X111.453 Y115.5 E.03577
"""


def scrub(footer: str) -> str:
    """Removes the two pieces of third-party content from a real footer.

    1. the model file name, read out of the footer itself (see
       ``STOP_PRINTING_OBJECT``) and replaced with ``PLACEHOLDER_NAME``;
    2. the model footprint outline inside PrusaSlicer's
       ``; objects_info = {...}`` line (see ``PLACEHOLDER_POLYGON``).

    Every other byte is verbatim.  Raises if the name cannot be found, so a
    footer this does not understand fails loudly instead of being committed
    with the name still in it.
    """
    names = {m.group("name") for m in STOP_PRINTING_OBJECT.finditer(footer)}
    if not names:
        raise SystemExit(
            "no '; stop printing object <name> id:' line: cannot locate the "
            "model name to scrub, refusing to write the fixture"
        )
    # Longest first, so a name that is a prefix of another cannot leave a
    # fragment behind.
    for name in sorted(names, key=len, reverse=True):
        footer = footer.replace(name, PLACEHOLDER_NAME)
    return replace_objects_info_polygon(footer)


def placeholder_polygon() -> str:
    """An obviously-synthetic 20x20 mm square traversed at 5 mm spacing.

    Serialized exactly as PrusaSlicer serializes an outline (``[x,y]``
    pairs at three decimal places, comma separated, no spaces), so the
    line keeps its real shape: long, and dense with commas and brackets.
    Nothing about it is model geometry -- an axis-aligned square whose
    edges carry evenly spaced collinear vertices is not a shape any
    slicer would ever emit for a real part.
    """
    points = []
    for x in range(100, 121, 5):
        points.append((x, 100))
    for y in range(105, 121, 5):
        points.append((120, y))
    for x in range(115, 99, -5):
        points.append((x, 120))
    for y in range(115, 104, -5):
        points.append((100, y))
    return ",".join("[{:.3f},{:.3f}]".format(x, y) for x, y in points)


PLACEHOLDER_POLYGON = placeholder_polygon()

# `; objects_info = {"objects":[{"name":...,"polygon":[[x,y],...]}]}`
OBJECTS_INFO_POLYGON = re.compile(r'("polygon":\[)(.*?)(\]\})')


def replace_objects_info_polygon(footer: str) -> str:
    """Swaps the real footprint outline for ``PLACEHOLDER_POLYGON``.

    The key names, the brace/bracket nesting and the numeric formatting
    are preserved; only the coordinates change. See the module docstring
    for why the real outline is not committed.
    """
    return OBJECTS_INFO_POLYGON.sub(
        lambda m: m.group(1) + PLACEHOLDER_POLYGON + m.group(3), footer
    )


CURA_BODY = """\
;FLAVOR:Marlin
;TIME:412
;Filament used: 1.2m
;Layer height: 0.2
; Fixture: Cura-style file WITH a footer, fully synthetic.
;
; Exercises: the completion gate on a third dialect, and -- uniquely in this
; corpus -- a footer that contains EXTRUSION-MODE COMMANDS. Cura's stock end
; g-code switches to relative positioning for the wipe-out move and back
; (`G91` ... `G90`), and because the effective extruder frame is
; `absolute_coord && absolute_extrude`, those are exactly the commands
; plr-analyzer's trust check has to reason about.
;Generated with Cura_SteamEngine 5.7.0
M82 ;absolute extrusion mode
G92 E0
M104 S200
M140 S60
M109 S200
M190 S60
G28 ;Home
G1 Z2.0 F3000
;LAYER_COUNT:2
;LAYER:0
M107
;TYPE:WALL-OUTER
G0 F9000 X60 Y60 Z0.2
G1 F1200 X80 Y60 E0.6221
G1 X80 Y80 E1.2442
G1 X60 Y80 E1.8663
G1 X60 Y60 E2.4884
;LAYER:1
;TYPE:WALL-OUTER
G0 F9000 X60 Y60 Z0.4
G1 F1200 X80 Y60 E3.1105
G1 X80 Y80 E3.7326
"""

CURA_FOOTER = """\
;TIME_ELAPSED:412.000000
G1 F1500 E3.2326
M107
M104 S0
M140 S0
;Retract the filament
G92 E1
G1 E-1 F300
G91 ;relative positioning
G0 F15000 X8.98 Y0.22 Z0.5 ;Wipe out
G0 Z10 ;Move up
G90 ;absolute positioning
M84 X Y E ;Disable steppers
"""


def cura_config_block() -> "list[str]":
    """Cura appends its settings as a base64-ish `;SETTING_3` comment run."""
    keys = padded_keys()
    lines = [";SETTING_3 {}"]
    lines += ["; {} = {}".format(k, v) for k, v in keys]
    lines.append(";End of Gcode")
    return lines


def write(name: str, text: str) -> None:
    path = os.path.join(HERE, name)
    with open(path, "w", newline="\n") as handle:
        handle.write(text)
    size = len(text.encode("utf-8"))
    marker = text.index(LAST_DEPOSITION_MARKER)
    print(
        "{}: {} bytes; {} follow the last depositing line".format(
            name, size, size - marker
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="regenerate footer fixtures")
    parser.add_argument(
        "--footer-dir",
        help="directory holding prusa_footer_raw.txt / orca_footer_raw.txt",
    )
    args = parser.parse_args()

    prusa = (
        PRUSA_BODY
        + LAST_DEPOSITION_MARKER
        + "\n"
        + PRUSA_SYNTHETIC_FOOTER
        + "\n".join(prusa_config_block())
        + "\n"
    )
    orca = (
        ORCA_BODY
        + LAST_DEPOSITION_MARKER
        + "\n"
        + ORCA_SYNTHETIC_FOOTER
        + "\n".join(orca_config_block())
        + "\n"
    )
    cura = (
        CURA_BODY
        + LAST_DEPOSITION_MARKER
        + "\n"
        + CURA_FOOTER
        + "\n".join(cura_config_block())
        + "\n"
    )
    write("prusa_footer_complete.gcode", prusa)
    write("orca_footer_complete.gcode", orca)
    write("cura_footer_complete.gcode", cura)

    if not args.footer_dir:
        print("(no --footer-dir: real-footer fixtures left untouched)")
        return
    for raw, body, out in (
        ("prusa_footer_raw.txt", PRUSA_REAL_BODY, "prusa_real_footer.gcode"),
        ("orca_footer_raw.txt", ORCA_REAL_BODY, "orca_real_footer.gcode"),
    ):
        with open(os.path.join(args.footer_dir, raw), newline="") as handle:
            footer = scrub(handle.read())
        print("  {}: real footer is {} bytes".format(out, len(footer.encode("utf-8"))))
        write(out, body + LAST_DEPOSITION_MARKER + "\n" + footer)


if __name__ == "__main__":
    main()
