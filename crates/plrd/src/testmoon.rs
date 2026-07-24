//! In-process fake Moonraker WebSocket server for tests.
//!
//! Loopback only, by policy: executor tests must NEVER touch a real
//! printer or a real Moonraker instance (one exists on this network).
//! The fake binds `127.0.0.1:0`, speaks just enough JSON-RPC 2.0 for
//! the client under test, records every call, and pushes a gratuitous
//! notification before each response so clients prove they skip them.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::Message;

/// Handler: `(method, params)` → `Ok(result)` or `Err((code, message))`.
pub type Handler = dyn Fn(&str, &Value) -> Result<Value, (i64, String)> + Send + Sync;

/// A running fake Moonraker.
pub struct FakeMoonraker {
    addr: SocketAddr,
    calls: Arc<Mutex<Vec<(String, Value)>>>,
    accept_task: tokio::task::JoinHandle<()>,
}

impl Drop for FakeMoonraker {
    fn drop(&mut self) {
        self.accept_task.abort();
    }
}

impl FakeMoonraker {
    /// Binds a loopback listener and serves `handler` on every
    /// connection (sequential reconnects supported).
    pub async fn spawn(
        handler: impl Fn(&str, &Value) -> Result<Value, (i64, String)> + Send + Sync + 'static,
    ) -> Self {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0")
            .await
            .expect("bind loopback");
        let addr = listener.local_addr().expect("local addr");
        let calls: Arc<Mutex<Vec<(String, Value)>>> = Arc::new(Mutex::new(Vec::new()));
        let handler: Arc<Handler> = Arc::new(handler);
        let calls_task = Arc::clone(&calls);
        let accept_task = tokio::spawn(async move {
            loop {
                let Ok((stream, _)) = listener.accept().await else {
                    return;
                };
                let handler = Arc::clone(&handler);
                let calls = Arc::clone(&calls_task);
                tokio::spawn(async move {
                    let _ = serve_connection(stream, &handler, &calls).await;
                });
            }
        });
        Self {
            addr,
            calls,
            accept_task,
        }
    }

    /// The `ws://` URL of this fake.
    #[must_use]
    pub fn url(&self) -> String {
        format!("ws://{}/websocket", self.addr)
    }

    /// Every `(method, params)` received so far, in order, across all
    /// connections.
    #[must_use]
    pub fn calls(&self) -> Vec<(String, Value)> {
        self.calls.lock().expect("calls lock").clone()
    }

    /// The gcode scripts received via `printer.gcode.script`, in order.
    #[must_use]
    pub fn gcode_sent(&self) -> Vec<String> {
        self.calls()
            .into_iter()
            .filter(|(method, _)| method == "printer.gcode.script")
            .filter_map(|(_, params)| {
                params
                    .get("script")
                    .and_then(Value::as_str)
                    .map(str::to_owned)
            })
            .collect()
    }
}

async fn serve_connection(
    stream: tokio::net::TcpStream,
    handler: &Arc<Handler>,
    calls: &Arc<Mutex<Vec<(String, Value)>>>,
) -> Result<(), tokio_tungstenite::tungstenite::Error> {
    use futures_util::{SinkExt as _, StreamExt as _};
    let mut ws = tokio_tungstenite::accept_async(stream).await?;
    while let Some(frame) = ws.next().await {
        let Message::Text(text) = frame? else {
            continue;
        };
        let Ok(request) = serde_json::from_str::<Value>(&text) else {
            continue;
        };
        let id = request.get("id").cloned().unwrap_or(Value::Null);
        let method = request
            .get("method")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_owned();
        let params = request.get("params").cloned().unwrap_or(Value::Null);
        calls
            .lock()
            .expect("calls lock")
            .push((method.clone(), params.clone()));
        // A gratuitous notification first: clients must skip it.
        let notify = json!({
            "jsonrpc": "2.0",
            "method": "notify_proc_stat_update",
            "params": [{"cpu": 1.0}],
        });
        ws.send(Message::text(notify.to_string())).await?;
        let response = match handler(&method, &params) {
            Ok(result) => json!({"jsonrpc": "2.0", "result": result, "id": id}),
            Err((code, message)) => json!({
                "jsonrpc": "2.0",
                "error": {"code": code, "message": message},
                "id": id,
            }),
        };
        ws.send(Message::text(response.to_string())).await?;
    }
    Ok(())
}
