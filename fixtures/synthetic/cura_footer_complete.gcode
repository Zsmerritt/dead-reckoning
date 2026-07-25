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
; THE LAST DEPOSITING LINE IS ABOVE THIS COMMENT
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
;SETTING_3 {}
; avoid_crossing_perimeters = 0
; bed_shape = 0x0,250x0,250x210,0x210
; bed_temperature = 60
; before_layer_gcode = ;BEFORE_LAYER_CHANGE\nG92 E0.0\n;[layer_z]\n\n
; between_objects_gcode = 
; bottom_fill_pattern = monotonic
; bottom_solid_layers = 4
; bottom_solid_min_thickness = 0.5
; bridge_acceleration = 1000
; bridge_angle = 0
; bridge_fan_speed = 100
; bridge_flow_ratio = 1
; bridge_speed = 25
; brim_separation = 0
; brim_type = outer_only
; brim_width = 0
; clip_multipart_objects = 1
; colorprint_heights = 
; complete_objects = 0
; cooling = 1
; cooling_tube_length = 5
; cooling_tube_retraction = 91.5
; default_acceleration = 1000
; default_filament_profile = Prusament PLA
; default_print_profile = 0.20mm QUALITY @MK3
; deretract_speed = 0
; disable_fan_first_layers = 1
; dont_support_bridges = 1
; draft_shield = disabled
; duplicate_distance = 6
; elefant_foot_compensation = 0.2
; end_filament_gcode = "; Filament-specific end gcode"
; external_perimeter_extrusion_width = 0.45
; external_perimeter_speed = 25
; external_perimeters_first = 0
; extra_loading_move = -2
; extra_perimeters = 0
; extruder_clearance_height = 20
; extruder_clearance_radius = 45
; extruder_colour = 
; extruder_offset = 0x0
; extrusion_axis = E
; extrusion_multiplier = 1
; extrusion_width = 0.45
; fan_always_on = 1
; fan_below_layer_time = 100
; filament_colour = #FF8000
; filament_cooling_final_speed = 3.4
; filament_cooling_initial_speed = 2.2
; filament_cooling_moves = 4
; filament_cost = 24.99
; filament_density = 1.24
; filament_deretract_speed = nil
; filament_diameter = 1.75
; filament_load_time = 0
; filament_loading_speed = 28
; filament_loading_speed_start = 3
; filament_max_volumetric_speed = 15
; filament_minimal_purge_on_wipe_tower = 15
; filament_notes = ""
; filament_ramming_parameters = "120 100 6.6 6.8 7.2 7.6 7.9 8.2 8.7 9.4 9.9"
; filament_retract_before_travel = nil
; filament_retract_before_wipe = nil
; filament_retract_layer_change = nil
; filament_retract_length = nil
; filament_retract_lift = nil
; filament_retract_restart_extra = nil
; filament_retract_speed = nil
; filament_settings_id = "Prusament PLA"
; filament_soluble = 0
; filament_spool_weight = 201
; filament_toolchange_delay = 0
; filament_type = PLA
; filament_unload_time = 0
; filament_unloading_speed = 90
; filament_unloading_speed_start = 100
; filament_vendor = Prusa Polymers
; fill_angle = 45
; fill_density = 15%
; fill_pattern = gyroid
; first_layer_acceleration = 800
; first_layer_acceleration_over_raft = 0
; first_layer_bed_temperature = 60
; first_layer_extrusion_width = 0.42
; first_layer_height = 0.2
; first_layer_speed = 20
; first_layer_speed_over_raft = 30
; first_layer_temperature = 215
; full_fan_speed_layer = 4
; fuzzy_skin = none
; fuzzy_skin_point_dist = 0.8
; fuzzy_skin_thickness = 0.3
; gap_fill_enabled = 1
; gap_fill_speed = 40
; gcode_comments = 0
; gcode_flavor = marlin2
; gcode_label_objects = 1
; gcode_resolution = 0.0125
; gcode_substitutions = 
; high_current_on_filament_swap = 0
; host_type = octoprint
; infill_acceleration = 1000
; infill_anchor = 2.5
; infill_anchor_max = 12
; infill_every_layers = 1
; infill_extruder = 1
; infill_extrusion_width = 0.45
; infill_first = 0
; infill_only_where_needed = 0
; infill_overlap = 25%
; infill_speed = 200
; interface_shells = 0
; ironing = 0
; ironing_flowrate = 15%
; ironing_spacing = 0.1
; ironing_speed = 15
; ironing_type = top
; layer_gcode = ;AFTER_LAYER_CHANGE\n;[layer_z]
; layer_height = 0.2
; machine_limits_usage = emit_to_gcode
; machine_max_acceleration_e = 5000,5000
; machine_max_acceleration_extruding = 1250,1250
; machine_max_acceleration_retracting = 1250,1250
; machine_max_acceleration_travel = 1500,1250
; machine_max_acceleration_x = 1000,960
; machine_max_acceleration_y = 1000,960
; machine_max_acceleration_z = 1000,1000
; machine_max_feedrate_e = 120,120
; machine_max_feedrate_x = 200,100
; machine_max_feedrate_y = 200,100
; machine_max_feedrate_z = 12,12
; machine_max_jerk_e = 4.5,4.5
; machine_max_jerk_x = 8,8
; machine_max_jerk_y = 8,8
; machine_max_jerk_z = 0.4,0.4
; machine_min_extruding_rate = 0,0
; machine_min_travel_rate = 0,0
; max_fan_speed = 100
; max_layer_height = 0.25
; max_print_height = 210
; max_print_speed = 200
; max_volumetric_extrusion_rate_slope_negative = 0
; max_volumetric_extrusion_rate_slope_positive = 0
; max_volumetric_speed = 0
; min_bead_width = 85%
; min_fan_speed = 100
; min_feature_size = 25%
; min_layer_height = 0.07
; min_print_speed = 15
; min_skirt_length = 4
; mmu_segmented_region_max_width = 0
; notes = 
; nozzle_diameter = 0.4
; nozzle_high_flow = 0
; only_retract_when_crossing_perimeters = 0
; ooze_prevention = 0
; output_filename_format = {input_filename_base}_{layer_height}mm.gcode
; overhangs = 1
; parking_pos_retraction = 92
; pause_print_gcode = M601
; perimeter_acceleration = 800
; perimeter_extruder = 1
; perimeter_extrusion_width = 0.45
; perimeter_generator = arachne
; perimeter_speed = 45
; perimeters = 2
; physical_printer_settings_id = 
; post_process = 
; print_settings_id = 0.20mm QUALITY @MK3
; printer_model = MK3S
; printer_notes = Don't remove the following keywords!
; printer_settings_id = Original Prusa i3 MK3S & MK3S+
; printer_technology = FFF
; printer_variant = 0.4
; printer_vendor = 
; raft_contact_distance = 0.1
; raft_expansion = 1.5
; raft_first_layer_density = 90%
; raft_first_layer_expansion = 3
; raft_layers = 0
; remaining_times = 1
; resolution = 0
; retract_before_travel = 1
; retract_before_wipe = 70%
; retract_layer_change = 1
; retract_length = 0.8
; retract_length_toolchange = 4
; retract_lift = 0.4
; retract_lift_above = 0
; retract_lift_below = 209
; retract_restart_extra = 0
; retract_restart_extra_toolchange = 0
; retract_speed = 35
; seam_position = aligned
; silent_mode = 1
; single_extruder_multi_material = 0
; single_extruder_multi_material_priming = 0
; skirt_distance = 2
; skirt_height = 3
; skirts = 0
; slice_closing_radius = 0.049
; slicing_mode = regular
; slowdown_below_layer_time = 20
; small_perimeter_speed = 25
; solid_infill_below_area = 0
; solid_infill_every_layers = 0
; solid_infill_extruder = 1
; solid_infill_extrusion_width = 0.45
; solid_infill_speed = 200
; spiral_vase = 0
; staggered_inner_seams = 0
; standby_temperature_delta = -5
; start_filament_gcode = "M900 K0.05 ; Filament gcode LA 1.5"
; support_material = 0
; support_material_angle = 0
; support_material_auto = 1
; support_material_bottom_contact_distance = 0
; support_material_bottom_interface_layers = -1
; support_material_buildplate_only = 0
; support_material_closing_radius = 2
; support_material_contact_distance = 0.1
; support_material_enforce_layers = 0
; support_material_extruder = 0
; support_material_extrusion_width = 0.35
; support_material_interface_contact_loops = 0
; support_material_interface_extruder = 0
; support_material_interface_layers = 2
; support_material_interface_pattern = rectilinear
; support_material_interface_spacing = 0.2
; support_material_interface_speed = 80%
; support_material_pattern = rectilinear
; support_material_spacing = 2
; support_material_speed = 50
; support_material_style = snug
; support_material_synchronize_layers = 0
; support_material_threshold = 55
; support_material_with_sheath = 0
; support_material_xy_spacing = 50%
; temperature = 210
; template_custom_gcode = 
; thick_bridges = 0
; thin_walls = 0
; threads = 12
; thumbnails = 16x16/QOI, 313x173/QOI
; toolchange_gcode = 
; top_fill_pattern = monotoniclines
; top_infill_extrusion_width = 0.4
; top_solid_infill_acceleration = 1000
; top_solid_infill_speed = 40
; top_solid_layers = 5
; top_solid_min_thickness = 0.7
; travel_acceleration = 1250
; travel_speed = 180
; travel_speed_z = 12
; use_firmware_retraction = 0
; use_relative_e_distances = 0
; use_volumetric_e = 0
; variable_layer_height = 1
; wall_distribution_count = 1
; wall_transition_angle = 10
; wall_transition_filter_deviation = 25%
; wall_transition_length = 100%
; wipe = 1
; wipe_into_infill = 0
; wipe_into_objects = 0
; wipe_tower = 1
; wipe_tower_bridging = 10
; wipe_tower_brim_width = 2
; wipe_tower_cone_angle = 0
; wipe_tower_extra_spacing = 100%
; wipe_tower_no_sparse_layers = 0
; wipe_tower_rotation_angle = 0
; wipe_tower_width = 60
; wipe_tower_x = 170
; wipe_tower_y = 125
; wiping_volumes_extruders = 70,70
; wiping_volumes_matrix = 0
; xy_size_compensation = 0
; z_offset = 0
; filament_custom_gcode_override_00 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_01 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_02 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_03 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_04 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_05 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_06 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_07 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_08 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_09 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_10 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_11 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_12 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_13 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_14 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_15 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_16 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_17 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_18 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_19 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_20 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_21 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_22 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_23 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_24 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_25 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_26 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_27 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_28 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_29 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_30 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_31 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_32 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_33 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_34 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_35 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_36 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_37 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_38 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_39 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_40 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_41 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_42 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_43 = "M900 K0.05 ; per-extruder linear-advance override"
; filament_custom_gcode_override_44 = "M900 K0.05 ; per-extruder linear-advance override"
;End of Gcode
