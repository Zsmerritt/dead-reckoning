//! The Klipper API-socket client task (tokio, Linux).
//!
//! # Session lifecycle
//!
//! ```text
//! connect → info (wait for state "ready") → objects/query heaters
//!         → objects/subscribe (§3 set + discovered heaters)
//!         → dump_trapq per queue → dump_stepper per Z stepper
//!         → read loop (+ periodic info poll)
//! ```
//!
//! On any socket error after the subscriptions were established, the
//! client journals a `SocketLost` marker (immediate durability), pauses
//! heartbeats (`WalCmd::Heartbeat(None)` — no liveness claim without a
//! live socket), and reconnects with capped exponential backoff. The
//! first successful resubscription after a loss journals `Resubscribed`.
//!
//! Subscribing to unconfigured objects is safe: Klipper's
//! `QueryStatusHelper._do_query` maps unknown objects to `{}` rather
//! than erroring, so the full §3 set is always requested and simply
//! yields nothing on printers without e.g. `z_thermal_adjust`.
//!
//! Klippy state is watched two ways: the startup gate polls `info` until
//! `state == "ready"` before subscribing, and a periodic `info` poll
//! during the session logs state transitions (a klippy `shutdown` keeps
//! the socket and the subscriptions alive — motion just stops, which the
//! reconstruction's quiet-tail classification is designed to read; an
//! actual RESTART closes the socket and takes the reconnect path).

use std::collections::VecDeque;
use std::io;
use std::path::PathBuf;
use std::time::Duration;

use plr_klipper::{
    classify, FrameEvent, FrameSplitter, Inbound, InfoResponse, Request, SubscriptionObjects,
};
use plr_wal::{Marker, MarkerKind};
use serde_json::Value;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;

use crate::convert::{route_of, status_template, stepper_template, trapq_template, Recorder};
use crate::hostclock::now_mono_ns;
use crate::sender::{WalGone, WalSender};

/// Fixed request ids for the session setup sequence.
const ID_INFO: u64 = 1;
const ID_HEATERS_QUERY: u64 = 2;
const ID_SUBSCRIBE: u64 = 3;
const ID_TRAPQ_BASE: u64 = 10;
const ID_STEPPER_BASE: u64 = 30;
/// Periodic info polls use ids from here upward.
const ID_POLL_BASE: u64 = 1_000;

/// The §3 status subscription set (minus per-printer heater objects,
/// which are discovered at runtime). `None` selects every field.
const STATUS_OBJECTS: &[&str] = &[
    "gcode_move",
    "toolhead",
    "virtual_sdcard",
    "mcu",
    "heaters",
    "fan",
    "bed_mesh",
    "exclude_object",
    "z_thermal_adjust",
    "skew_correction",
    "idle_timeout",
    "probe",
];

/// Client configuration (derived from `config::Config`).
#[derive(Debug, Clone)]
pub struct ClientCfg {
    /// Klipper API socket path.
    pub socket: PathBuf,
    /// Trapq names to dump.
    pub trapq_queues: Vec<String>,
    /// Z stepper names to dump.
    pub z_steppers: Vec<String>,
    /// Interval between info polls inside a live session.
    pub info_poll: Duration,
    /// Backoff cap for reconnect attempts.
    pub backoff_cap: Duration,
    /// Initial backoff after a failure.
    pub backoff_initial: Duration,
}

impl ClientCfg {
    /// Default timing knobs around a socket path and the configured dump
    /// lists.
    #[must_use]
    pub fn new(socket: PathBuf, trapq_queues: Vec<String>, z_steppers: Vec<String>) -> Self {
        Self {
            socket,
            trapq_queues,
            z_steppers,
            info_poll: Duration::from_secs(2),
            backoff_cap: Duration::from_secs(8),
            backoff_initial: Duration::from_millis(250),
        }
    }
}

/// Runs the client forever (or until the WAL thread dies, the only
/// fatal error). Cancellation-safe: the daemon drops this future on
/// shutdown.
pub async fn run_client(
    cfg: &ClientCfg,
    sender: &mut WalSender,
    recorder: &mut Recorder,
) -> Result<(), WalGone> {
    let mut backoff = cfg.backoff_initial;
    let mut lost_after_subscribe = false;
    loop {
        match UnixStream::connect(&cfg.socket).await {
            Ok(stream) => {
                let mut conn = Conn::new(stream);
                let loss = session(cfg, &mut conn, sender, recorder, lost_after_subscribe).await?;
                eprintln!("plrd: klipper session ended: {}", loss.error);
                if loss.subscribed {
                    // Motion after this instant was not observed.
                    sender.marker(Marker {
                        mono_ns: now_mono_ns(),
                        kind: MarkerKind::SocketLost,
                    })?;
                    // No liveness claim without a live socket, and no
                    // cross-session state diffs (see `reset_session`).
                    sender.heartbeat_data(None)?;
                    recorder.reset_session();
                    lost_after_subscribe = true;
                    backoff = cfg.backoff_initial;
                }
            }
            Err(e) => {
                eprintln!(
                    "plrd: cannot connect to {}: {e}; retrying in {backoff:?}",
                    cfg.socket.display()
                );
            }
        }
        tokio::time::sleep(backoff).await;
        backoff = (backoff * 2).min(cfg.backoff_cap);
    }
}

/// Why a session ended (always an I/O-shaped reason; `WalGone` is
/// propagated separately as the only fatal error).
struct SessionLoss {
    /// The subscriptions had been established when the session died.
    subscribed: bool,
    /// The underlying error.
    error: io::Error,
}

/// One connected session: setup, then the read loop. Returns how the
/// session was lost.
async fn session(
    cfg: &ClientCfg,
    conn: &mut Conn,
    sender: &mut WalSender,
    recorder: &mut Recorder,
    resubscription: bool,
) -> Result<SessionLoss, WalGone> {
    match setup(cfg, conn, sender, recorder).await {
        Ok(()) => {}
        Err(SetupError::Io(error)) => {
            return Ok(SessionLoss {
                subscribed: false,
                error,
            })
        }
        Err(SetupError::Fatal(gone)) => return Err(gone),
    }
    if resubscription {
        sender.marker(Marker {
            mono_ns: now_mono_ns(),
            kind: MarkerKind::Resubscribed,
        })?;
    }
    match read_loop(cfg, conn, sender, recorder).await {
        Ok(error) => Ok(SessionLoss {
            subscribed: true,
            error,
        }),
        Err(gone) => Err(gone),
    }
}

enum SetupError {
    Io(io::Error),
    Fatal(WalGone),
}

impl From<io::Error> for SetupError {
    fn from(e: io::Error) -> Self {
        SetupError::Io(e)
    }
}

/// Info gate + heater discovery + subscriptions.
async fn setup(
    cfg: &ClientCfg,
    conn: &mut Conn,
    sender: &mut WalSender,
    recorder: &mut Recorder,
) -> Result<(), SetupError> {
    // 1. Wait for klippy "ready": subscribing during startup would give
    //    incomplete object sets.
    let mut info_id = ID_INFO;
    loop {
        conn.send(&Request::Info { client_info: None }, info_id)
            .await?;
        let result = await_result(conn, info_id, sender, recorder).await?;
        let info: InfoResponse = serde_json::from_value(result).map_err(io::Error::other)?;
        if info.state.as_deref() == Some("ready") {
            break;
        }
        eprintln!(
            "plrd: klippy state {:?}; waiting for ready",
            info.state.as_deref().unwrap_or("unknown")
        );
        tokio::time::sleep(Duration::from_secs(1)).await;
        info_id = info_id.wrapping_add(2);
        if info_id >= ID_POLL_BASE {
            info_id = ID_INFO;
        }
    }

    // 2. Discover heater objects.
    let mut objects = SubscriptionObjects::new();
    objects.insert("heaters".to_owned(), None);
    conn.send(&Request::ObjectsQuery { objects }, ID_HEATERS_QUERY)
        .await?;
    let result = await_result(conn, ID_HEATERS_QUERY, sender, recorder).await?;
    let heater_names = available_heaters(&result);
    recorder.set_heater_names(heater_names.clone());

    // 3. Subscribe to the status set. The response body is the full
    //    initial status; journal it as the baseline context.
    let mut objects = SubscriptionObjects::new();
    for name in STATUS_OBJECTS {
        objects.insert((*name).to_owned(), None);
    }
    for name in heater_names {
        objects.insert(name, None);
    }
    conn.send(
        &Request::ObjectsSubscribe {
            objects,
            response_template: Some(status_template()),
        },
        ID_SUBSCRIBE,
    )
    .await?;
    let result = await_result(conn, ID_SUBSCRIBE, sender, recorder).await?;
    let mono_ns = now_mono_ns();
    match recorder.on_initial_status(&result, mono_ns) {
        Ok(out) => forward(out, sender, mono_ns).map_err(SetupError::Fatal)?,
        Err(e) => eprintln!("plrd: initial status unparseable: {e}"),
    }

    // 4. Motion dumps, each with a distinct template key.
    for (i, queue) in cfg.trapq_queues.iter().enumerate() {
        let id = ID_TRAPQ_BASE + i as u64;
        conn.send(
            &Request::DumpTrapq {
                name: queue.clone(),
                response_template: Some(trapq_template(queue)),
            },
            id,
        )
        .await?;
        let _header = await_result(conn, id, sender, recorder).await?;
    }
    for (i, stepper) in cfg.z_steppers.iter().enumerate() {
        let id = ID_STEPPER_BASE + i as u64;
        conn.send(
            &Request::DumpStepper {
                name: stepper.clone(),
                response_template: Some(stepper_template(stepper)),
            },
            id,
        )
        .await?;
        let _header = await_result(conn, id, sender, recorder).await?;
    }
    Ok(())
}

/// The steady-state loop: dispatch notifications, poll info. Returns the
/// I/O error that ended it.
async fn read_loop(
    cfg: &ClientCfg,
    conn: &mut Conn,
    sender: &mut WalSender,
    recorder: &mut Recorder,
) -> Result<io::Error, WalGone> {
    let mut poll = tokio::time::interval(cfg.info_poll);
    poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    poll.reset(); // do not fire immediately
    let mut poll_id = ID_POLL_BASE;
    let mut last_state: Option<String> = None;
    loop {
        tokio::select! {
            inbound = conn.recv() => match inbound {
                Err(e) => return Ok(e),
                Ok(Inbound::Notification(n)) => dispatch(&n, sender, recorder)?,
                Ok(Inbound::Response { id, result }) if id >= ID_POLL_BASE => {
                    track_klippy_state(&result, &mut last_state);
                }
                Ok(Inbound::Response { .. }) => {}
                Ok(Inbound::Error { id, error }) => {
                    eprintln!("plrd: klipper error for request {id}: {error:?}");
                }
            },
            _ = poll.tick() => {
                poll_id = poll_id.wrapping_add(1).max(ID_POLL_BASE);
                if let Err(e) = conn.send(&Request::Info { client_info: None }, poll_id).await {
                    return Ok(e);
                }
            }
        }
    }
}

/// Routes one notification through the recorder and forwards its output.
fn dispatch(
    notification: &plr_klipper::Notification,
    sender: &mut WalSender,
    recorder: &mut Recorder,
) -> Result<(), WalGone> {
    let Some(route) = route_of(&notification.template) else {
        return Ok(()); // not one of ours
    };
    let mono_ns = now_mono_ns();
    match recorder.on_notification(&route, notification, mono_ns) {
        Ok(out) => forward(out, sender, mono_ns),
        Err(e) => {
            eprintln!("plrd: unparseable {route:?} payload: {e}");
            Ok(())
        }
    }
}

/// Pushes one conversion output into the WAL channel.
fn forward(
    out: crate::convert::Output,
    sender: &mut WalSender,
    mono_ns: u64,
) -> Result<(), WalGone> {
    for (record, sync) in out.records {
        sender.record(record, sync, mono_ns)?;
    }
    if let Some(hb) = out.heartbeat {
        sender.heartbeat_data(Some(hb))?;
    }
    if let Some((obs_mono_ns, widened)) = out.receive_seq {
        sender.receive_seq(obs_mono_ns, widened)?;
    }
    if out.clean_shutdown {
        // The print ended on purpose; the WAL says so durably. See
        // convert.rs for exactly what this is keyed on.
        sender.marker(Marker {
            mono_ns,
            kind: MarkerKind::CleanShutdown,
        })?;
    }
    Ok(())
}

fn track_klippy_state(result: &Value, last_state: &mut Option<String>) {
    let state = result
        .get("state")
        .and_then(Value::as_str)
        .unwrap_or("unknown")
        .to_owned();
    if last_state.as_deref() != Some(&state) {
        if last_state.is_some() {
            eprintln!("plrd: klippy state changed to {state:?}");
        }
        *last_state = Some(state);
    }
}

/// Extracts `status.heaters.available_heaters` from an `objects/query`
/// result. Missing pieces yield an empty list (printer without heaters).
fn available_heaters(result: &Value) -> Vec<String> {
    result
        .get("status")
        .and_then(|s| s.get("heaters"))
        .and_then(|h| h.get("available_heaters"))
        .and_then(Value::as_array)
        .map(|names| {
            names
                .iter()
                .filter_map(Value::as_str)
                .map(str::to_owned)
                .collect()
        })
        .unwrap_or_default()
}

/// Waits for the response to `id`, dispatching any notifications that
/// arrive in between (subscriptions are already live during setup).
async fn await_result(
    conn: &mut Conn,
    id: u64,
    sender: &mut WalSender,
    recorder: &mut Recorder,
) -> Result<Value, SetupError> {
    loop {
        match conn.recv().await.map_err(SetupError::Io)? {
            Inbound::Response { id: got, result } if got == id => return Ok(result),
            Inbound::Error { id: got, error } if got == id => {
                return Err(SetupError::Io(io::Error::other(format!(
                    "klipper rejected request {id}: {:?}",
                    error.message
                ))))
            }
            // Replies to other requests (stale polls) are not ours.
            Inbound::Response { .. } | Inbound::Error { .. } => {}
            Inbound::Notification(n) => {
                dispatch(&n, sender, recorder).map_err(SetupError::Fatal)?;
            }
        }
    }
}

/// Socket + frame splitter + classified-message queue.
struct Conn {
    stream: UnixStream,
    splitter: FrameSplitter,
    inbox: VecDeque<Inbound>,
    buf: Vec<u8>,
}

impl Conn {
    fn new(stream: UnixStream) -> Self {
        Self {
            stream,
            splitter: FrameSplitter::new(),
            inbox: VecDeque::new(),
            buf: vec![0; 64 * 1024],
        }
    }

    async fn send(&mut self, request: &Request, id: u64) -> io::Result<()> {
        let frame = request.to_frame(id).map_err(io::Error::other)?;
        self.stream.write_all(&frame).await
    }

    async fn recv(&mut self) -> io::Result<Inbound> {
        loop {
            if let Some(inbound) = self.inbox.pop_front() {
                return Ok(inbound);
            }
            let n = self.stream.read(&mut self.buf).await?;
            if n == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "klipper closed the socket",
                ));
            }
            for event in self.splitter.feed(&self.buf[..n]) {
                match event {
                    FrameEvent::Frame(frame) => match classify(&frame) {
                        Ok(inbound) => self.inbox.push_back(inbound),
                        Err(e) => eprintln!("plrd: unclassifiable frame: {e}"),
                    },
                    FrameEvent::Oversized { discarded_len } => {
                        eprintln!("plrd: oversized frame discarded ({discarded_len} bytes)");
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{available_heaters, run_client, track_klippy_state, ClientCfg};
    use crate::convert::Recorder;
    use crate::sender::{WalCmd, WalSender};
    use plr_wal::{MarkerKind, WalRecord};
    use serde_json::{json, Value};
    use std::path::PathBuf;
    use std::sync::mpsc::Receiver;
    use std::time::Duration;
    use tokio::io::{AsyncReadExt, AsyncWriteExt};
    use tokio::net::{UnixListener, UnixStream};

    fn temp_sock(tag: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!(
            "plrd-client-test-{tag}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir.join("klippy.sock")
    }

    /// Server-side connection: reads `{id, method, params}` requests and
    /// sends raw JSON frames.
    struct Server {
        stream: UnixStream,
        buf: Vec<u8>,
        pending: Vec<u8>,
    }

    impl Server {
        fn new(stream: UnixStream) -> Self {
            Self {
                stream,
                buf: vec![0; 16 * 1024],
                pending: Vec::new(),
            }
        }

        async fn read_request(&mut self) -> (u64, String, Value) {
            loop {
                if let Some(pos) = self.pending.iter().position(|&b| b == 0x03) {
                    let frame: Vec<u8> = self.pending.drain(..=pos).collect();
                    let value: Value = serde_json::from_slice(&frame[..frame.len() - 1]).unwrap();
                    let id = value["id"].as_u64().unwrap();
                    let method = value["method"].as_str().unwrap().to_owned();
                    return (id, method, value["params"].clone());
                }
                let n = self.stream.read(&mut self.buf).await.unwrap();
                assert!(n > 0, "client hung up mid-setup");
                self.pending.extend_from_slice(&self.buf[..n]);
            }
        }

        async fn send(&mut self, value: &Value) {
            let mut bytes = serde_json::to_vec(value).unwrap();
            bytes.push(0x03);
            self.stream.write_all(&bytes).await.unwrap();
        }

        async fn respond(&mut self, id: u64, result: Value) {
            self.send(&json!({"id": id, "result": result})).await;
        }
    }

    fn initial_status() -> Value {
        json!({
            "eventtime": 100.0,
            "status": {
                "toolhead": {"print_time": 10.0, "estimated_print_time": 9.5,
                              "position": [5.0, 6.0, 0.2, 100.0]},
                "gcode_move": {
                    "speed_factor": 1.0, "speed": 1500.0, "extrude_factor": 1.0,
                    "absolute_coordinates": true, "absolute_extrude": false,
                    "homing_origin": [0.0, 0.0, 0.0, 0.0],
                    "position": [5.0, 6.0, 0.2, 100.0],
                    "gcode_position": [5.0, 6.0, 0.2, 100.0]
                },
                "virtual_sdcard": {"file_path": "/g/x.gcode", "progress": 0.5,
                                    "is_active": true, "file_position": 1000,
                                    "file_size": 2000},
                "extruder": {"target": 215.0},
                "fan": {"speed": 0.75},
                "mcu": {"last_stats": {"receive_seq": 4100}},
            }
        })
    }

    /// Serves one complete setup sequence (info → heaters → subscribe →
    /// 2 trapq + 1 stepper dumps), asserting the requests' shape.
    async fn serve_setup(server: &mut Server) {
        let (id, method, _params) = server.read_request().await;
        assert_eq!(method, "info");
        server.respond(id, json!({"state": "ready"})).await;

        let (id, method, params) = server.read_request().await;
        assert_eq!(method, "objects/query");
        assert_eq!(params["objects"], json!({"heaters": null}));
        server
            .respond(
                id,
                json!({"eventtime": 99.0, "status": {"heaters": {
                    "available_heaters": ["extruder"],
                    "available_sensors": ["extruder"]}}}),
            )
            .await;

        let (id, method, params) = server.read_request().await;
        assert_eq!(method, "objects/subscribe");
        assert_eq!(params["response_template"], json!({"k": "status"}));
        let objects = params["objects"].as_object().unwrap();
        // The §3 set plus the discovered heater.
        for key in [
            "gcode_move",
            "toolhead",
            "virtual_sdcard",
            "mcu",
            "heaters",
            "fan",
            "bed_mesh",
            "exclude_object",
            "z_thermal_adjust",
            "skew_correction",
            "idle_timeout",
            "probe",
            "extruder",
        ] {
            assert!(objects.contains_key(key), "subscribe missing {key}");
            assert!(objects[key].is_null(), "{key} must request all fields");
        }
        server.respond(id, initial_status()).await;

        for expected in [
            ("motion_report/dump_trapq", "toolhead", "trapq:toolhead"),
            ("motion_report/dump_trapq", "extruder", "trapq:extruder"),
            (
                "motion_report/dump_stepper",
                "stepper_z",
                "stepper:stepper_z",
            ),
        ] {
            let (id, method, params) = server.read_request().await;
            assert_eq!(method, expected.0);
            assert_eq!(params["name"], json!(expected.1));
            assert_eq!(params["response_template"]["k"], json!(expected.2));
            server.respond(id, json!({"header": []})).await;
        }
    }

    /// Kinds of interest extracted from the WAL channel, in order.
    fn interesting(rx: &Receiver<WalCmd>) -> Vec<String> {
        let mut out = Vec::new();
        while let Ok(cmd) = rx.try_recv() {
            match cmd {
                WalCmd::Append { record, sync } => match record {
                    WalRecord::Context(_) => out.push(format!("context/{sync:?}")),
                    WalRecord::TrapqSegment(s) => out.push(format!("trapq/{}", s.queue)),
                    WalRecord::StepperRange(r) => out.push(format!("stepper/{}", r.stepper)),
                    WalRecord::Marker(m) => out.push(format!("marker/{:?}", kind_name(&m.kind))),
                    WalRecord::Heartbeat(_) => out.push("wal-heartbeat".to_owned()),
                },
                WalCmd::Heartbeat(Some(_)) => out.push("hb-data".to_owned()),
                WalCmd::Heartbeat(None) => out.push("hb-pause".to_owned()),
                WalCmd::ReceiveSeq { widened, .. } => out.push(format!("seq/{widened}")),
                WalCmd::Shutdown => out.push("shutdown".to_owned()),
            }
        }
        out
    }

    fn kind_name(kind: &MarkerKind) -> &'static str {
        match kind {
            MarkerKind::CleanShutdown => "CleanShutdown",
            MarkerKind::SocketLost => "SocketLost",
            MarkerKind::Resubscribed => "Resubscribed",
            MarkerKind::SubscriptionGap { .. } => "SubscriptionGap",
            MarkerKind::Unknown => "Unknown",
        }
    }

    fn position_of(items: &[String], needle: &str) -> usize {
        items
            .iter()
            .position(|i| i == needle)
            .unwrap_or_else(|| panic!("`{needle}` not found in {items:?}"))
    }

    #[tokio::test]
    async fn full_session_reconnect_and_marker_flow() {
        let sock = temp_sock("full");
        let listener = UnixListener::bind(&sock).unwrap();
        let (tx, rx) = std::sync::mpsc::sync_channel(1024);
        let mut cfg = ClientCfg::new(
            sock.clone(),
            vec!["toolhead".into(), "extruder".into()],
            vec!["stepper_z".into()],
        );
        cfg.backoff_initial = Duration::from_millis(10);
        cfg.backoff_cap = Duration::from_millis(50);

        let client = tokio::spawn(async move {
            let mut sender = WalSender::new(tx);
            let mut recorder = Recorder::new();
            let _ = run_client(&cfg, &mut sender, &mut recorder).await;
        });

        let scenario = async {
            // --- Session 1 ---
            let (stream, _) = listener.accept().await.unwrap();
            let mut server = Server::new(stream);
            serve_setup(&mut server).await;
            // A trapq batch notification.
            server
                .send(&json!({"k": "trapq:toolhead", "params": {"data": [
                    [12.5, 0.25, 40.0, -1500.0, [10.0, 20.0, 0.3], [1.0, 0.0, 0.0]]
                ]}}))
                .await;
            // A stepper batch notification.
            server
                .send(&json!({"k": "stepper:stepper_z", "params": {
                    "data": [[7457, 1, 0]],
                    "start_position": 0.2, "start_mcu_position": 80,
                    "step_distance": 0.0025,
                    "first_clock": 5_000_000_000_u64, "first_step_time": 27.7,
                    "last_clock": 5_000_100_000_u64, "last_step_time": 27.71
                }}))
                .await;
            // Print completes: file position reaches the end, reader
            // stops. The client must journal CleanShutdown.
            server
                .send(
                    &json!({"k": "status", "params": {"eventtime": 101.0, "status": {
                        "virtual_sdcard": {"file_position": 2000, "progress": 1.0,
                                            "is_active": false}
                    }}}),
                )
                .await;
            // An unroutable notification must be ignored, not crash.
            server
                .send(&json!({"k": "mystery:x", "params": {"whatever": 1}}))
                .await;
            // Give the client time to drain, then drop the connection.
            tokio::time::sleep(Duration::from_millis(100)).await;
            drop(server);

            // --- Session 2 (reconnect) ---
            let (stream, _) = listener.accept().await.unwrap();
            let mut server = Server::new(stream);
            serve_setup(&mut server).await;
            tokio::time::sleep(Duration::from_millis(100)).await;
            server
        };
        let _server = tokio::time::timeout(Duration::from_secs(10), scenario)
            .await
            .expect("scenario timed out");
        client.abort();
        let _ = client.await;

        let items = interesting(&rx);
        // Session 1 setup: baseline context (immediate) + receive_seq.
        let first_context = position_of(&items, "context/Immediate");
        position_of(&items, "seq/4100"); // receive_seq observation persisted
                                         // Motion flowed.
        let trapq = position_of(&items, "trapq/toolhead");
        let stepper = position_of(&items, "stepper/stepper_z");
        // Print end was journaled.
        let clean = position_of(&items, "marker/\"CleanShutdown\"");
        // Socket loss: marker + heartbeat pause, then resubscription.
        let lost = position_of(&items, "marker/\"SocketLost\"");
        let pause = position_of(&items, "hb-pause");
        let resub = position_of(&items, "marker/\"Resubscribed\"");
        assert!(first_context < trapq, "{items:?}");
        assert!(trapq < clean, "{items:?}");
        assert!(stepper < clean, "{items:?}");
        assert!(clean < lost, "{items:?}");
        assert!(lost < pause, "{items:?}");
        assert!(pause < resub, "{items:?}");
        // The reconnect journaled a second baseline context (the marker
        // lands after it: "Resubscribed" is only true once the new
        // subscriptions — whose response carries that context — stand).
        assert!(
            items[pause..resub].iter().any(|i| i == "context/Immediate"),
            "{items:?}"
        );
        // Heartbeat data flowed at some point before the loss.
        assert!(items[..lost].iter().any(|i| i == "hb-data"), "{items:?}");
    }

    #[tokio::test]
    async fn waits_for_klippy_ready_before_subscribing() {
        let sock = temp_sock("notready");
        let listener = UnixListener::bind(&sock).unwrap();
        let (tx, rx) = std::sync::mpsc::sync_channel(64);
        let mut cfg = ClientCfg::new(sock.clone(), vec!["toolhead".into()], vec![]);
        cfg.backoff_initial = Duration::from_millis(10);

        let client = tokio::spawn(async move {
            let mut sender = WalSender::new(tx);
            let mut recorder = Recorder::new();
            let _ = run_client(&cfg, &mut sender, &mut recorder).await;
        });

        let scenario = async {
            let (stream, _) = listener.accept().await.unwrap();
            let mut server = Server::new(stream);
            // First info: still starting up. The client must poll again
            // rather than subscribe.
            let (id, method, _) = server.read_request().await;
            assert_eq!(method, "info");
            server.respond(id, json!({"state": "startup"})).await;
            let (id, method, _) = server.read_request().await;
            assert_eq!(method, "info");
            server.respond(id, json!({"state": "ready"})).await;
            let (id, method, _) = server.read_request().await;
            assert_eq!(method, "objects/query");
            server
                .respond(
                    id,
                    json!({"eventtime": 1.0, "status": {"heaters":
                        {"available_heaters": [], "available_sensors": []}}}),
                )
                .await;
            let (id, method, _) = server.read_request().await;
            assert_eq!(method, "objects/subscribe");
            server.respond(id, initial_status()).await;
            let (id, method, _) = server.read_request().await;
            assert_eq!(method, "motion_report/dump_trapq");
            server.respond(id, json!({"header": []})).await;
            tokio::time::sleep(Duration::from_millis(50)).await;
            server
        };
        let _server = tokio::time::timeout(Duration::from_secs(10), scenario)
            .await
            .expect("scenario timed out");
        client.abort();
        let _ = client.await;
        let items = interesting(&rx);
        assert!(
            items.iter().any(|i| i == "context/Immediate"),
            "no baseline context after delayed ready: {items:?}"
        );
    }

    #[test]
    fn available_heaters_parses_and_defaults() {
        let names = available_heaters(&json!({"eventtime": 1.0, "status": {"heaters": {
            "available_heaters": ["extruder", "heater_bed", 42],
        }}}));
        assert_eq!(names, ["extruder", "heater_bed"]);
        assert!(available_heaters(&json!({})).is_empty());
        assert!(available_heaters(&json!({"status": {}})).is_empty());
    }

    #[test]
    fn klippy_state_transitions_are_tracked() {
        let mut last = None;
        track_klippy_state(&json!({"state": "ready"}), &mut last);
        assert_eq!(last.as_deref(), Some("ready"));
        track_klippy_state(&json!({"state": "ready"}), &mut last);
        assert_eq!(last.as_deref(), Some("ready"));
        track_klippy_state(&json!({"state": "shutdown"}), &mut last);
        assert_eq!(last.as_deref(), Some("shutdown"));
        track_klippy_state(&json!({}), &mut last);
        assert_eq!(last.as_deref(), Some("unknown"));
    }
}
