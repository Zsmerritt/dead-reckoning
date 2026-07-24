//! End-to-end fixture tests: realistic wire frames (bytes as Klipper
//! sends them, ETX-terminated) run through the splitter, classifier and
//! typed payload parsers.
//!
//! Each fixture's shape is cited to the Klipper source that emits it.

// Test-only pedantic relaxations, each deliberate:
// - doc_markdown: fixture docs quote Klipper identifiers/paths as prose;
//   backticking every one would bury the citations in noise.
// - unreadable_literal / float_cmp: fixture values mirror the wire bytes
//   exactly, and asserts check the exact parsed values.
// - too_many_lines / similar_names: the full-subscription fixture is one
//   deliberate end-to-end scenario over many similarly named objects.
#![allow(
    clippy::doc_markdown,
    clippy::unreadable_literal,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::similar_names
)]

use plr_klipper::{classify, FrameEvent, FrameSplitter, Inbound, Notification};

/// Splits `bytes` and classifies the single frame it must contain.
fn one_frame(bytes: &[u8]) -> Inbound {
    let mut splitter = FrameSplitter::new();
    let mut events = splitter.feed(bytes);
    assert_eq!(events.len(), 1, "expected exactly one frame");
    let FrameEvent::Frame(frame) = events.remove(0) else {
        panic!("expected a frame event");
    };
    classify(&frame).expect("fixture frame must classify")
}

fn notification(bytes: &[u8]) -> Notification {
    match one_frame(bytes) {
        Inbound::Notification(n) => n,
        other => panic!("expected notification, got {other:?}"),
    }
}

/// `info` response — klippy/webhooks.py, WebHooks._handle_info_request
/// (keys: state, state_message, hostname, klipper_path, python_path,
/// process_id, user_id, group_id + start args log_file, config_file,
/// software_version, cpu_info).
#[test]
fn info_response_fixture() {
    let bytes = br#"{"id": 1, "result": {"state": "ready", "state_message": "Printer is ready", "hostname": "voron", "klipper_path": "/home/pi/klipper", "python_path": "/home/pi/klippy-env/bin/python", "process_id": 872, "user_id": 1000, "group_id": 1000, "log_file": "/home/pi/printer_data/logs/klippy.log", "config_file": "/home/pi/printer_data/config/printer.cfg", "software_version": "v0.13.0-462-g7046bd00e", "cpu_info": "4 core ?"}}"#;
    let mut framed = bytes.to_vec();
    framed.push(3);
    let Inbound::Response { id, result } = one_frame(&framed) else {
        panic!("expected response");
    };
    assert_eq!(id, 1);
    let info: plr_klipper::InfoResponse = serde_json::from_value(result).unwrap();
    assert_eq!(info.state.as_deref(), Some("ready"));
    assert_eq!(info.process_id, Some(872));
    assert_eq!(
        info.software_version.as_deref(),
        Some("v0.13.0-462-g7046bd00e")
    );
}

/// `objects/subscribe` initial response — klippy/webhooks.py,
/// QueryStatusHelper._handle_query sends {'eventtime':..., 'status':...};
/// object payloads per toolhead.py ToolHead.get_status,
/// kinematics/cartesian.py get_status, extras/gcode_move.py get_status,
/// extras/virtual_sdcard.py get_status, mcu.py MCUStatsHelper,
/// extras/heaters.py Heater.get_status, extras/fan.py Fan.get_status,
/// extras/bed_mesh.py update_status, extras/exclude_object.py get_status,
/// extras/z_thermal_adjust.py get_status, extras/skew_correction.py
/// get_status, extras/idle_timeout.py get_status, extras/probe.py
/// ProbeCommandHelper.get_status.
#[test]
fn objects_subscribe_response_fixture() {
    let json = serde_json::json!({
        "id": 2,
        "result": {
            "eventtime": 3052153.382083195,
            "status": {
                "webhooks": {"state": "ready", "state_message": "Printer is ready"},
                "toolhead": {
                    "print_time": 812.512, "estimated_print_time": 812.498,
                    "homed_axes": "xyz", "position": [150.0, 150.0, 5.2, 1042.7],
                    "extruder": "extruder",
                    "max_velocity": 300.0, "max_accel": 3000.0
                },
                "gcode_move": {
                    "speed_factor": 1.0, "speed": 9000.0, "extrude_factor": 0.95,
                    "absolute_coordinates": true, "absolute_extrude": false,
                    "homing_origin": [0.0, 0.0, -0.05, 0.0],
                    "position": [150.0, 150.0, 5.2, 1042.7],
                    "gcode_position": [150.0, 150.0, 5.25, 12.5]
                },
                "virtual_sdcard": {
                    "file_path": "/home/pi/printer_data/gcodes/benchy.gcode",
                    "progress": 0.4133, "is_active": true,
                    "file_position": 1123456, "file_size": 2718281
                },
                "mcu": {
                    "mcu_version": "v0.13.0-462-g7046bd00e",
                    "mcu_build_versions": "gcc: (GCC) 10.3.1",
                    "mcu_constants": {"CLOCK_FREQ": 180000000, "MCU": "stm32f446xx",
                                      "STATS_SUMSQ_BASE": 256},
                    // last_stats per mcu.py MCUStatsHelper.stats: the stats
                    // line parsed with float-if-dot-else-int values; serial
                    // keys per chelper/serialqueue.c serialqueue_get_stats.
                    "last_stats": {
                        "mcu_awake": 0.032, "mcu_task_avg": 0.000012,
                        "mcu_task_stddev": 0.000024,
                        "bytes_write": 3271941, "bytes_read": 6531017,
                        "bytes_retransmit": 9, "bytes_invalid": 0,
                        "send_seq": 194913, "receive_seq": 194913,
                        "retransmit_seq": 2,
                        "srtt": 0.021, "rttvar": 0.005, "rto": 0.025,
                        "ready_bytes": 0, "upcoming_bytes": 0,
                        "freq": 180002214
                    }
                },
                "extruder": {"temperature": 245.03, "target": 245.0, "power": 0.6238},
                "heater_bed": {"temperature": 100.01, "target": 100.0, "power": 0.3},
                "fan": {"speed": 1.0, "rpm": null},
                "bed_mesh": {
                    "profile_name": "",
                    "mesh_min": [20.0, 20.0], "mesh_max": [280.0, 280.0],
                    "probed_matrix": [[0.01, 0.02], [0.0, -0.01]],
                    "mesh_matrix": [[0.01, 0.015, 0.02], [0.005, 0.005, 0.005],
                                    [0.0, -0.005, -0.01]],
                    "profiles": {"default": {}}
                },
                "exclude_object": {
                    "objects": [
                        {"name": "BENCHY.STL", "center": [150.0, 150.0],
                         "polygon": [[140.0, 140.0], [160.0, 140.0],
                                     [160.0, 160.0], [140.0, 160.0]]}
                    ],
                    "excluded_objects": [],
                    "current_object": "BENCHY.STL"
                },
                "z_thermal_adjust": {
                    "temperature": 41.3, "measured_min_temp": 22.1,
                    "measured_max_temp": 43.7, "current_z_adjust": -0.0123,
                    "z_adjust_ref_temperature": 30.0, "enabled": true
                },
                "skew_correction": {"current_profile_name": "calilantern"},
                "idle_timeout": {"state": "Printing", "printing_time": 1523.4,
                                  "idle_timeout": 600.0},
                "probe": {"name": "probe", "last_query": false,
                          "last_probe_position": [150.0, 150.0, 1.985, 0.0],
                          "last_z_result": 2.015}
            }
        }
    });
    let mut framed = serde_json::to_vec(&json).unwrap();
    framed.push(3);
    let Inbound::Response { id: 2, result } = one_frame(&framed) else {
        panic!("expected response id 2");
    };
    let update: plr_klipper::StatusUpdate = serde_json::from_value(result).unwrap();
    let status = &update.status;

    let th = status.toolhead().unwrap().unwrap();
    assert_eq!(th.homed_axes.as_deref(), Some("xyz"));
    assert_eq!(th.print_time, Some(812.512));
    assert_eq!(th.estimated_print_time, Some(812.498));

    let gm = status.gcode_move().unwrap().unwrap();
    assert_eq!(gm.homing_origin, Some(vec![0.0, 0.0, -0.05, 0.0]));
    assert_eq!(gm.absolute_extrude, Some(false));

    let sd = status.virtual_sdcard().unwrap().unwrap();
    assert_eq!(
        sd.file_path,
        Some(Some("/home/pi/printer_data/gcodes/benchy.gcode".to_owned()))
    );
    assert_eq!(sd.file_position, Some(1_123_456));

    let mcu = status.mcu().unwrap().unwrap();
    assert_eq!(mcu.clock_freq(), Some(180_000_000.0));
    let stats = mcu.last_stats.unwrap();
    assert_eq!(stats.receive_seq, Some(194_913));
    assert_eq!(stats.freq, Some(180_002_214));
    assert_eq!(stats.srtt, Some(0.021));

    let hotend = status.heater("extruder").unwrap().unwrap();
    assert_eq!(hotend.target, Some(245.0));
    let bed = status.heater("heater_bed").unwrap().unwrap();
    assert_eq!(bed.power, Some(0.3));

    let fan = status.fan("fan").unwrap().unwrap();
    assert_eq!(fan.speed, Some(1.0));
    assert_eq!(fan.rpm, Some(None)); // null: no tachometer

    let mesh = status.bed_mesh().unwrap().unwrap();
    // Adaptive mesh: profile_name empty but matrix non-empty → active.
    assert_eq!(mesh.profile_name.as_deref(), Some(""));
    assert_eq!(mesh.mesh_active(), Some(true));

    let excl = status.exclude_object().unwrap().unwrap();
    assert_eq!(excl.current_object, Some(Some("BENCHY.STL".to_owned())));
    assert_eq!(excl.objects.unwrap()[0].name, "BENCHY.STL");

    let zta = status.z_thermal_adjust().unwrap().unwrap();
    assert_eq!(zta.current_z_adjust, Some(-0.0123));

    let skew = status.skew_correction().unwrap().unwrap();
    assert_eq!(skew.current_profile_name.as_deref(), Some("calilantern"));

    let idle = status.idle_timeout().unwrap().unwrap();
    assert_eq!(idle.state.as_deref(), Some("Printing"));

    let probe = status.probe().unwrap().unwrap();
    // Raw trigger Z = bed_z + z_offset (probe.py cmd_PROBE):
    // 1.985 + 0.030 = 2.015.
    assert_eq!(probe.last_z_result, Some(2.015));
    assert_eq!(
        probe.last_probe_position,
        Some(vec![150.0, 150.0, 1.985, 0.0])
    );
}

/// `objects/subscribe` async diff update — klippy/webhooks.py,
/// QueryStatusHelper._do_query sends only changed fields, wrapped in the
/// client's response_template.
#[test]
fn objects_subscribe_diff_update_fixture() {
    let bytes = b"{\"params\": {\"eventtime\": 3052153.632112, \"status\": {\"toolhead\": {\"print_time\": 812.760}, \"virtual_sdcard\": {\"file_position\": 1123999, \"progress\": 0.4135}}}, \"q\": \"status\"}\x03";
    let n = notification(bytes);
    assert_eq!(n.template.get("q"), Some(&serde_json::json!("status")));
    let update = n.status_update().unwrap();
    let th = update.status.toolhead().unwrap().unwrap();
    // Only the changed field is present.
    assert_eq!(th.print_time, Some(812.760));
    assert_eq!(th.estimated_print_time, None);
    assert_eq!(th.position, None);
    let sd = update.status.virtual_sdcard().unwrap().unwrap();
    assert_eq!(sd.file_position, Some(1_123_999));
    assert_eq!(sd.file_path, None); // absent ≠ null
}

/// `virtual_sdcard` with no file loaded — extras/virtual_sdcard.py,
/// file_path() returns None → JSON null.
#[test]
fn virtual_sdcard_null_file_fixture() {
    let bytes = b"{\"params\": {\"eventtime\": 100.0, \"status\": {\"virtual_sdcard\": {\"file_path\": null, \"progress\": 0.0, \"is_active\": false, \"file_position\": 0, \"file_size\": 0}}}}\x03";
    let update = notification(bytes).status_update().unwrap();
    let sd = update.status.virtual_sdcard().unwrap().unwrap();
    assert_eq!(sd.file_path, Some(None)); // present and null
    assert_eq!(sd.is_active, Some(false));
}

/// Inactive `bed_mesh` baseline — extras/bed_mesh.py, update_status sets
/// probed_matrix/mesh_matrix to [[]] and profile_name to "" when no mesh
/// is loaded.
#[test]
fn bed_mesh_inactive_fixture() {
    let bytes = b"{\"params\": {\"eventtime\": 5.0, \"status\": {\"bed_mesh\": {\"profile_name\": \"\", \"mesh_min\": [0.0, 0.0], \"mesh_max\": [0.0, 0.0], \"probed_matrix\": [[]], \"mesh_matrix\": [[]], \"profiles\": {}}}}}\x03";
    let update = notification(bytes).status_update().unwrap();
    let mesh = update.status.bed_mesh().unwrap().unwrap();
    assert_eq!(mesh.mesh_active(), Some(false));
    // A diff without mesh_matrix cannot decide activity.
    let partial: plr_klipper::BedMeshStatus =
        serde_json::from_value(serde_json::json!({"profile_name": "default"})).unwrap();
    assert_eq!(partial.mesh_active(), None);
}

/// `motion_report/dump_trapq` subscription: response header then a batch —
/// extras/motion_report.py DumpTrapQ (api_resp header; _process_batch
/// row shape) via extras/bulk_sensor.py BatchBulkHelper /
/// BatchWebhooksClient.handle_batch.
#[test]
fn dump_trapq_fixture() {
    let mut splitter = FrameSplitter::new();
    let mut stream = Vec::new();
    stream.extend_from_slice(
        br#"{"id": 3, "result": {"header": ["time", "duration", "start_velocity", "acceleration", "start_position", "direction"]}}"#,
    );
    stream.push(3);
    stream.extend_from_slice(
        br#"{"params": {"data": [[812.512, 0.084, 0.0, 3000.0, [150.0, 150.0, 5.2], [0.7071, 0.7071, 0.0]], [812.596, 0.416, 251.9, 0.0, [158.9, 158.9, 5.2], [0.7071, 0.7071, 0.0]]]}, "q": "trapq:toolhead"}"#,
    );
    stream.push(3);
    let events = splitter.feed(&stream);
    assert_eq!(events.len(), 2);

    let FrameEvent::Frame(first) = &events[0] else {
        panic!("expected frame");
    };
    let Inbound::Response { id: 3, result } = classify(first).unwrap() else {
        panic!("expected response id 3");
    };
    let header: plr_klipper::DumpHeader = serde_json::from_value(result).unwrap();
    assert_eq!(header.header.len(), 6);
    assert_eq!(header.header[4], "start_position");

    let FrameEvent::Frame(second) = &events[1] else {
        panic!("expected frame");
    };
    let Inbound::Notification(n) = classify(second).unwrap() else {
        panic!("expected notification");
    };
    assert_eq!(
        n.template.get("q"),
        Some(&serde_json::json!("trapq:toolhead"))
    );
    let batch = n.trapq_batch().unwrap();
    assert_eq!(batch.data.len(), 2);
    assert_eq!(batch.data[0].time, 812.512);
    assert_eq!(batch.data[1].start_velocity, 251.9);
    assert_eq!(batch.data[1].start_position, [158.9, 158.9, 5.2]);
}

/// Extruder trapq batch: only the first position/direction component is
/// meaningful — extras/motion_report.py registers a DumpTrapQ per
/// extruder ("extruder", "extruder1", ...).
#[test]
fn dump_trapq_extruder_fixture() {
    let bytes = b"{\"params\": {\"data\": [[812.512, 0.084, 4.2, 0.0, [1042.7, 0.0, 0.0], [1.0, 0.0, 0.0]]]}, \"q\": \"trapq:extruder\"}\x03";
    let batch = notification(bytes).trapq_batch().unwrap();
    assert_eq!(batch.data[0].start_position[0], 1042.7);
    assert_eq!(batch.data[0].direction, [1.0, 0.0, 0.0]);
}

/// `motion_report/dump_stepper` subscription: response header then a
/// batch — extras/motion_report.py DumpStepper (api_resp header
/// ('interval', 'count', 'add'); _process_batch payload keys data,
/// start_position, start_mcu_position, step_distance, first_clock,
/// first_step_time, last_clock, last_step_time). Rows are the signed C
/// history fields (chelper/stepcompress.h struct pull_history_steps):
/// the batch mixes real captured values from a Trident-class triple-Z
/// machine — a wrapped-u32 first interval (-2136919700), forward rows,
/// and reverse-direction rows (count -40 and -1; stepcompress.c:372
/// negates count when stepping in reverse, as every Z lift/lower does).
#[test]
fn dump_stepper_fixture() {
    let mut splitter = FrameSplitter::new();
    let mut stream = Vec::new();
    stream.extend_from_slice(br#"{"id": 4, "result": {"header": ["interval", "count", "add"]}}"#);
    stream.push(3);
    stream.extend_from_slice(
        br#"{"params": {"data": [[-2136919700, 1, 0], [10000, 976, 0], [9855, -40, 187], [12000, -1, 0]], "start_position": 5.2, "start_mcu_position": 2080, "step_distance": 0.0025, "first_clock": 146258295000, "first_step_time": 812.546, "last_clock": 146268110855, "last_step_time": 812.600}, "q": "stepper:stepper_z"}"#,
    );
    stream.push(3);
    let events = splitter.feed(&stream);
    assert_eq!(events.len(), 2);

    let FrameEvent::Frame(first) = &events[0] else {
        panic!("expected frame");
    };
    let Inbound::Response { id: 4, result } = classify(first).unwrap() else {
        panic!("expected response id 4");
    };
    let header: plr_klipper::DumpHeader = serde_json::from_value(result).unwrap();
    assert_eq!(header.header, vec!["interval", "count", "add"]);

    let FrameEvent::Frame(second) = &events[1] else {
        panic!("expected frame");
    };
    let batch = match classify(second).unwrap() {
        Inbound::Notification(n) => n.stepper_batch().unwrap(),
        other => panic!("expected notification, got {other:?}"),
    };
    assert_eq!(batch.data.len(), 4);
    // Wrapped u32 interval on the first row after an idle period.
    assert_eq!(batch.data[0].interval, -2136919700);
    assert_eq!(batch.data[0].interval_ticks(), 2_158_047_596);
    assert_eq!(batch.data[1].count, 976);
    // Reverse-direction rows keep their magnitude in steps().
    assert_eq!(batch.data[2].count, -40);
    assert_eq!(batch.data[2].steps(), 40);
    assert_eq!(batch.data[2].add, 187);
    assert_eq!(batch.data[3].count, -1);
    assert_eq!(batch.start_mcu_position, 2080);
    assert_eq!(batch.first_clock, 146_258_295_000);
    // Cross-check against the tick conversion: 180 MHz MCU.
    let clock = plr_klipper::McuClock::new(180_000_000.0).unwrap();
    assert!((clock.clock_to_print_time(batch.first_clock) - batch.first_step_time).abs() < 1e-3);
}

/// `gcode/subscribe_output` message — klippy/webhooks.py,
/// GCodeHelper._output_callback; example shape per docs/API_Server.md.
#[test]
fn gcode_output_fixture() {
    let bytes = b"{\"params\": {\"response\": \"// Klipper state: Shutdown\"}, \"key\": 345}\x03";
    let n = notification(bytes);
    let out = n.gcode_output().unwrap();
    assert_eq!(out.response, "// Klipper state: Shutdown");
}

/// Error response — klippy/webhooks.py, WebRequest.finish +
/// WebRequestError.to_dict.
#[test]
fn error_response_fixture() {
    let bytes = b"{\"id\": 5, \"error\": {\"error\": \"WebRequestError\", \"message\": \"webhooks: No registered callback for path 'bogus'\"}}\x03";
    let Inbound::Error { id, error } = one_frame(bytes) else {
        panic!("expected error");
    };
    assert_eq!(id, 5);
    assert_eq!(error.error.as_deref(), Some("WebRequestError"));
    assert!(error.message.unwrap().contains("bogus"));
}

/// Clock correlation across a realistic status stream: toolhead status
/// supplies (eventtime, estimated_print_time) pairs; trapq times map into
/// host time between them.
#[test]
fn correlator_over_status_stream() {
    let mut correlator = plr_klipper::ClockCorrelator::new();
    for bytes in [
        b"{\"params\": {\"eventtime\": 3052153.382, \"status\": {\"toolhead\": {\"estimated_print_time\": 812.498}}}}\x03".as_slice(),
        b"{\"params\": {\"eventtime\": 3052153.632, \"status\": {\"toolhead\": {\"estimated_print_time\": 812.748}}}}\x03".as_slice(),
    ] {
        let update = notification(bytes).status_update().unwrap();
        if let Some(ept) = update.status.toolhead().unwrap().unwrap().estimated_print_time {
            assert_eq!(
                correlator.add_sample(update.eventtime, ept),
                plr_klipper::SampleOutcome::Accepted
            );
        }
    }
    let host = correlator.print_time_to_eventtime(812.760).unwrap();
    assert!((host - 3_052_153.644).abs() < 1e-9);
}
