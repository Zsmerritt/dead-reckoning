//! Moonraker WebSocket JSON-RPC client.
//!
//! # Protocol (Moonraker docs, cited per method)
//!
//! * Endpoint: `ws://host:port/websocket` — Moonraker docs,
//!   `external_api/introduction.md` ("JSON-RPC API Overview").
//! * Framing: JSON-RPC 2.0 — `{"jsonrpc": "2.0", "method": ...,
//!   "params": {...}, "id": <unique>}`; success carries `result`,
//!   failure `error: {code, message}` (same page).
//! * `printer.objects.query` — query printer object status; params
//!   `{"objects": {"name": null | [fields]}}`; result
//!   `{eventtime, status: {...}}` — `external_api/printer.md`.
//! * `printer.gcode.script` — run a G-Code script; params
//!   `{"script": ...}`; **the response returns when the script has
//!   completed** (or errored) — `external_api/printer.md`. This is what
//!   makes post-step verification sound: by the time a step's send
//!   resolves, its commands have run.
//! * `printer.info` — klippy state (`ready`/`shutdown`/...) —
//!   `external_api/printer.md`.
//!
//! Server-initiated JSON-RPC *notifications* (messages with a `method`
//! and no `id`, e.g. `notify_proc_stat_update`) are skipped while
//! waiting for a call's response.
//!
//! This client performs **read-only queries and gcode script execution
//! only** — no file APIs, no power APIs, no printer administration.

use std::time::Duration;

use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::Message;

/// Errors from the Moonraker client.
#[derive(Debug, thiserror::Error)]
pub enum MoonrakerError {
    /// TCP/WebSocket-level failure.
    #[error("moonraker connection: {0}")]
    Ws(#[from] tokio_tungstenite::tungstenite::Error),
    /// The server closed the stream mid-call.
    #[error("moonraker closed the connection")]
    Closed,
    /// The call did not complete within the timeout.
    #[error("moonraker call `{method}` timed out after {seconds}s")]
    Timeout {
        /// The JSON-RPC method that timed out.
        method: String,
        /// The timeout that fired, in seconds.
        seconds: u64,
    },
    /// JSON-RPC error response.
    #[error("moonraker error {code} for `{method}`: {message}")]
    Rpc {
        /// The JSON-RPC method that failed.
        method: String,
        /// JSON-RPC error code.
        code: i64,
        /// Server-supplied message.
        message: String,
    },
    /// A frame that was not valid JSON-RPC.
    #[error("moonraker protocol violation: {0}")]
    Protocol(String),
}

/// A connected Moonraker JSON-RPC client.
#[derive(Debug)]
pub struct MoonrakerClient {
    ws: tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    next_id: u64,
    /// Per-call timeout. G-code scripts can legitimately run for
    /// minutes (heating, probing); queries resolve in milliseconds.
    call_timeout: Duration,
}

impl MoonrakerClient {
    /// Connects to a Moonraker WebSocket endpoint
    /// (`ws://host:port/websocket`).
    pub async fn connect(url: &str, timeout: Duration) -> Result<Self, MoonrakerError> {
        let connect = tokio_tungstenite::connect_async(url);
        let (ws, _response) = tokio::time::timeout(timeout, connect).await.map_err(|_| {
            MoonrakerError::Timeout {
                method: "connect".to_owned(),
                seconds: timeout.as_secs(),
            }
        })??;
        Ok(Self {
            ws,
            next_id: 1,
            call_timeout: timeout,
        })
    }

    /// Sets the per-call timeout (defaults to the connect timeout).
    pub fn set_call_timeout(&mut self, timeout: Duration) {
        self.call_timeout = timeout;
    }

    /// One JSON-RPC call: sends the request, skips notifications, and
    /// returns the matching `result`.
    pub async fn call(&mut self, method: &str, params: Value) -> Result<Value, MoonrakerError> {
        let timeout = self.call_timeout;
        tokio::time::timeout(timeout, self.call_inner(method, params))
            .await
            .map_err(|_| MoonrakerError::Timeout {
                method: method.to_owned(),
                seconds: timeout.as_secs(),
            })?
    }

    async fn call_inner(&mut self, method: &str, params: Value) -> Result<Value, MoonrakerError> {
        use futures_util::{SinkExt as _, StreamExt as _};
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1).max(1);
        let mut request = json!({
            "jsonrpc": "2.0",
            "method": method,
            "id": id,
        });
        if !params.is_null() {
            request["params"] = params;
        }
        self.ws.send(Message::text(request.to_string())).await?;
        loop {
            let frame = match self.ws.next().await {
                None => return Err(MoonrakerError::Closed),
                Some(frame) => frame?,
            };
            let text = match frame {
                Message::Text(text) => text,
                // Server pings are answered by tungstenite internally;
                // any other frame kind is skipped.
                Message::Close(_) => return Err(MoonrakerError::Closed),
                _ => continue,
            };
            let value: Value = serde_json::from_str(&text)
                .map_err(|e| MoonrakerError::Protocol(format!("non-JSON frame: {e}")))?;
            // Notifications carry a method and no id: not ours.
            if value.get("id").is_none_or(Value::is_null) {
                continue;
            }
            if value["id"].as_u64() != Some(id) {
                // A response to a different (stale) call: skip.
                continue;
            }
            if let Some(error) = value.get("error") {
                return Err(MoonrakerError::Rpc {
                    method: method.to_owned(),
                    code: error.get("code").and_then(Value::as_i64).unwrap_or(0),
                    message: error
                        .get("message")
                        .and_then(Value::as_str)
                        .unwrap_or("unknown error")
                        .to_owned(),
                });
            }
            return value
                .get("result")
                .cloned()
                .ok_or_else(|| MoonrakerError::Protocol("response without result".to_owned()));
        }
    }

    /// Runs a G-Code script (`printer.gcode.script`). Resolves when the
    /// script has completed (Moonraker semantics; see module docs).
    pub async fn gcode_script(&mut self, script: &str) -> Result<(), MoonrakerError> {
        self.call("printer.gcode.script", json!({ "script": script }))
            .await
            .map(|_| ())
    }

    /// Queries full status of the named objects
    /// (`printer.objects.query`); returns the `status` map.
    pub async fn query_objects(&mut self, objects: &[&str]) -> Result<Value, MoonrakerError> {
        let mut map = serde_json::Map::new();
        for name in objects {
            map.insert((*name).to_owned(), Value::Null);
        }
        let result = self
            .call("printer.objects.query", json!({ "objects": map }))
            .await?;
        result
            .get("status")
            .cloned()
            .ok_or_else(|| MoonrakerError::Protocol("query result without status".to_owned()))
    }
}

#[cfg(test)]
mod tests {
    use super::{MoonrakerClient, MoonrakerError};
    use crate::testmoon::FakeMoonraker;
    use serde_json::json;
    use std::time::Duration;

    const T: Duration = Duration::from_secs(5);

    #[tokio::test]
    async fn call_round_trips_and_skips_notifications() {
        let fake = FakeMoonraker::spawn(|method, _params| match method {
            "printer.info" => Ok(json!({"state": "ready"})),
            other => Err((-32601, format!("Method not found: {other}"))),
        })
        .await;
        // The fake pushes a notification before every response; the
        // client must skip it.
        let mut client = MoonrakerClient::connect(&fake.url(), T).await.unwrap();
        let info = client
            .call("printer.info", serde_json::Value::Null)
            .await
            .unwrap();
        assert_eq!(info["state"], json!("ready"));
        // Ids advance and match across multiple calls.
        let info = client
            .call("printer.info", serde_json::Value::Null)
            .await
            .unwrap();
        assert_eq!(info["state"], json!("ready"));
        let calls = fake.calls();
        assert_eq!(calls.len(), 2);
        assert_eq!(calls[0].0, "printer.info");
    }

    #[tokio::test]
    async fn rpc_errors_map_to_typed_errors() {
        let fake = FakeMoonraker::spawn(|method, _| match method {
            "printer.gcode.script" => Err((400, "Klippy is shutdown".to_owned())),
            _ => Ok(json!("ok")),
        })
        .await;
        let mut client = MoonrakerClient::connect(&fake.url(), T).await.unwrap();
        let err = client.gcode_script("G28 X Y").await.unwrap_err();
        let MoonrakerError::Rpc {
            method,
            code,
            message,
        } = err
        else {
            panic!("expected Rpc error, got {err:?}");
        };
        assert_eq!(method, "printer.gcode.script");
        assert_eq!(code, 400);
        assert!(message.contains("shutdown"));
    }

    #[tokio::test]
    async fn query_objects_extracts_status() {
        let fake = FakeMoonraker::spawn(|method, params| {
            assert_eq!(method, "printer.objects.query");
            assert_eq!(params["objects"]["webhooks"], serde_json::Value::Null);
            Ok(json!({"eventtime": 1.0, "status": {"webhooks": {"state": "ready"}}}))
        })
        .await;
        let mut client = MoonrakerClient::connect(&fake.url(), T).await.unwrap();
        let status = client.query_objects(&["webhooks"]).await.unwrap();
        assert_eq!(status["webhooks"]["state"], json!("ready"));
    }

    #[tokio::test]
    async fn unreachable_endpoint_is_a_connect_error() {
        // Port 9 (discard) on localhost: connection refused.
        let err = MoonrakerClient::connect("ws://127.0.0.1:9/websocket", T).await;
        assert!(err.is_err());
    }

    #[tokio::test]
    async fn gcode_script_sends_the_exact_script() {
        let fake = FakeMoonraker::spawn(|_, _| Ok(json!("ok"))).await;
        let mut client = MoonrakerClient::connect(&fake.url(), T).await.unwrap();
        client
            .gcode_script("SET_IDLE_TIMEOUT TIMEOUT=86400")
            .await
            .unwrap();
        let calls = fake.calls();
        assert_eq!(calls[0].0, "printer.gcode.script");
        assert_eq!(
            calls[0].1["script"],
            json!("SET_IDLE_TIMEOUT TIMEOUT=86400")
        );
    }
}
