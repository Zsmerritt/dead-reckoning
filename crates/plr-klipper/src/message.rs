//! Classification of inbound frames and typed request-response payloads.
//!
//! Klipper sends exactly three top-level message shapes on the API socket
//! (`klippy/webhooks.py`):
//!
//! * `{"id": <echoed id>, "result": {...}}` — successful reply
//!   (`WebRequest.finish`).
//! * `{"id": <echoed id>, "error": {"error": "WebRequestError",
//!   "message": ...}}` — failed reply (`WebRequest.finish`,
//!   `WebRequestError.to_dict`).
//! * `{<response_template keys...>, "params": {...}}` — asynchronous
//!   message from a subscription or remote-method call (`docs/API_Server.md`
//!   "Subscriptions"; `bulk_sensor.BatchWebhooksClient.handle_batch`,
//!   `QueryStatusHelper._do_query`, `GCodeHelper._output_callback`).
//!
//! [`classify`] maps a frame body to that taxonomy without panicking on
//! arbitrary input.

use serde::Deserialize;
use serde_json::{Map, Value};

use crate::dump::{StepperBatch, TrapqBatch};
use crate::error::MessageError;
use crate::status::StatusUpdate;

/// A classified inbound message.
#[derive(Debug, Clone, PartialEq)]
pub enum Inbound {
    /// Successful reply to the request with the echoed `id`.
    Response {
        /// The client-chosen id echoed by Klipper.
        id: u64,
        /// The endpoint-specific `result` payload.
        result: Value,
    },
    /// Failed reply to the request with the echoed `id`.
    Error {
        /// The client-chosen id echoed by Klipper.
        id: u64,
        /// The error body.
        error: ApiError,
    },
    /// An asynchronous notification (subscription update, dump batch,
    /// G-Code output, or remote-method call).
    Notification(Notification),
}

/// Error body of a failed reply (`klippy/webhooks.py`,
/// `WebRequestError.to_dict`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct ApiError {
    /// Error class name; `"WebRequestError"` in current Klipper.
    pub error: Option<String>,
    /// Human-readable error message.
    pub message: Option<String>,
}

/// An asynchronous message: the client's `response_template` keys with an
/// added `params` payload (`docs/API_Server.md`, "Subscriptions").
#[derive(Debug, Clone, PartialEq)]
pub struct Notification {
    /// The endpoint-specific payload from the `params` key.
    /// [`Value::Null`] if the message had no `params` key (possible only
    /// if a template shadowed it or the peer misbehaves).
    pub params: Value,
    /// All remaining top-level keys: the echo of the `response_template`
    /// given at subscription time. Use a distinct key per subscription to
    /// route notifications.
    pub template: Map<String, Value>,
}

impl Notification {
    /// Parses `params` as any deserializable payload, labelling errors
    /// with `context`.
    pub fn params_as<T: serde::de::DeserializeOwned>(
        &self,
        context: &str,
    ) -> Result<T, MessageError> {
        serde_json::from_value(self.params.clone()).map_err(|source| MessageError::Payload {
            context: context.to_owned(),
            source,
        })
    }

    /// Parses `params` as an `objects/subscribe` status update.
    pub fn status_update(&self) -> Result<StatusUpdate, MessageError> {
        self.params_as("objects/subscribe update")
    }

    /// Parses `params` as a `motion_report/dump_trapq` batch.
    pub fn trapq_batch(&self) -> Result<TrapqBatch, MessageError> {
        self.params_as("motion_report/dump_trapq batch")
    }

    /// Parses `params` as a `motion_report/dump_stepper` batch.
    pub fn stepper_batch(&self) -> Result<StepperBatch, MessageError> {
        self.params_as("motion_report/dump_stepper batch")
    }

    /// Parses `params` as a `gcode/subscribe_output` message.
    pub fn gcode_output(&self) -> Result<GcodeOutput, MessageError> {
        self.params_as("gcode/subscribe_output message")
    }
}

/// Payload of a `gcode/subscribe_output` notification
/// (`klippy/webhooks.py`, `GCodeHelper._output_callback` sets
/// `params = {'response': msg}`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct GcodeOutput {
    /// One line of G-Code terminal output.
    pub response: String,
}

/// `result` payload of the `info` endpoint (`klippy/webhooks.py`,
/// `WebHooks._handle_info_request`). Fields are optional because the
/// start-arg derived entries (`log_file`, `config_file`,
/// `software_version`, `cpu_info`) may be `null` and future Klipper
/// versions may drop or add keys.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct InfoResponse {
    /// Printer state: `"ready"`, `"startup"`, `"shutdown"`, or `"error"`.
    pub state: Option<String>,
    /// Human-readable state message.
    pub state_message: Option<String>,
    /// Host name of the machine running Klipper.
    pub hostname: Option<String>,
    /// Path to the Klipper source directory.
    pub klipper_path: Option<String>,
    /// Path to the python executable running Klipper.
    pub python_path: Option<String>,
    /// Process id of the Klipper host process.
    pub process_id: Option<u64>,
    /// User id the Klipper host process runs as.
    pub user_id: Option<u64>,
    /// Group id the Klipper host process runs as.
    pub group_id: Option<u64>,
    /// Path to the Klipper log file (start arg; may be null).
    pub log_file: Option<String>,
    /// Path to the printer config file (start arg; may be null).
    pub config_file: Option<String>,
    /// Klipper software version string (start arg; may be null).
    pub software_version: Option<String>,
    /// CPU description (start arg; may be null).
    pub cpu_info: Option<String>,
}

/// `result` payload of `motion_report/dump_trapq` and
/// `motion_report/dump_stepper` subscription requests: the column names
/// for later `data` rows (`klippy/extras/motion_report.py`, the
/// `api_resp` dictionaries; `klippy/extras/bulk_sensor.py`,
/// `BatchBulkHelper._add_api_client`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct DumpHeader {
    /// Column names describing each `data` row.
    pub header: Vec<String>,
}

/// Classifies one frame body (as yielded by
/// [`FrameSplitter`](crate::frame::FrameSplitter), terminator excluded).
///
/// Total on arbitrary bytes: returns an error rather than panicking on
/// non-JSON input, non-object messages, or protocol violations.
pub fn classify(frame: &[u8]) -> Result<Inbound, MessageError> {
    let value: Value = serde_json::from_slice(frame)?;
    let Value::Object(mut map) = value else {
        return Err(MessageError::NotAnObject);
    };
    if let Some(id_value) = map.remove("id") {
        let id = id_value.as_u64().ok_or(MessageError::InvalidId)?;
        if let Some(result) = map.remove("result") {
            return Ok(Inbound::Response { id, result });
        }
        if let Some(error) = map.remove("error") {
            let error = serde_json::from_value(error).map_err(|source| MessageError::Payload {
                context: "error body".to_owned(),
                source,
            })?;
            return Ok(Inbound::Error { id, error });
        }
        return Err(MessageError::MissingResultOrError { id });
    }
    let params = map.remove("params").unwrap_or(Value::Null);
    Ok(Inbound::Notification(Notification {
        params,
        template: map,
    }))
}

#[cfg(test)]
mod tests {
    use super::{classify, ApiError, Inbound};
    use crate::error::MessageError;
    use serde_json::json;

    #[test]
    fn classifies_result_response() {
        let inbound = classify(br#"{"id": 123, "result": {"state": "ready"}}"#).unwrap();
        assert_eq!(
            inbound,
            Inbound::Response {
                id: 123,
                result: json!({"state": "ready"}),
            }
        );
    }

    #[test]
    fn classifies_error_response() {
        // Shape per webhooks.py WebRequestError.to_dict.
        let inbound = classify(
            br#"{"id": 4, "error": {"error": "WebRequestError",
                 "message": "Must home axis first"}}"#,
        )
        .unwrap();
        assert_eq!(
            inbound,
            Inbound::Error {
                id: 4,
                error: ApiError {
                    error: Some("WebRequestError".to_owned()),
                    message: Some("Must home axis first".to_owned()),
                },
            }
        );
    }

    #[test]
    fn classifies_notification_with_template_echo() {
        // Shape per docs/API_Server.md "Subscriptions" example.
        let inbound =
            classify(br#"{"params": {"response": "ok T:22.4 /0.0"}, "key": 345}"#).unwrap();
        let Inbound::Notification(n) = inbound else {
            panic!("expected notification");
        };
        assert_eq!(n.params, json!({"response": "ok T:22.4 /0.0"}));
        assert_eq!(n.template.get("key"), Some(&json!(345)));
        assert_eq!(n.gcode_output().unwrap().response, "ok T:22.4 /0.0");
    }

    #[test]
    fn notification_without_params_yields_null_params() {
        let inbound = classify(br#"{"key": 1}"#).unwrap();
        let Inbound::Notification(n) = inbound else {
            panic!("expected notification");
        };
        assert!(n.params.is_null());
        // ...and parsing it as a typed payload fails cleanly.
        let err = n.gcode_output().unwrap_err();
        assert!(matches!(err, MessageError::Payload { .. }));
        assert!(err.to_string().contains("gcode/subscribe_output"));
    }

    #[test]
    fn rejects_invalid_json() {
        assert!(matches!(
            classify(b"{\"id\": 1,").unwrap_err(),
            MessageError::Json(_)
        ));
        assert!(matches!(
            classify(&[0xff, 0xfe]).unwrap_err(),
            MessageError::Json(_)
        ));
    }

    #[test]
    fn rejects_non_object_top_level() {
        assert!(matches!(
            classify(b"[1, 2, 3]").unwrap_err(),
            MessageError::NotAnObject
        ));
        assert!(matches!(
            classify(b"42").unwrap_err(),
            MessageError::NotAnObject
        ));
    }

    #[test]
    fn rejects_non_integer_id() {
        for frame in [
            br#"{"id": "abc", "result": {}}"#.as_slice(),
            br#"{"id": -1, "result": {}}"#.as_slice(),
            br#"{"id": 1.5, "result": {}}"#.as_slice(),
        ] {
            assert!(matches!(
                classify(frame).unwrap_err(),
                MessageError::InvalidId
            ));
        }
    }

    #[test]
    fn rejects_id_without_result_or_error() {
        let err = classify(br#"{"id": 9}"#).unwrap_err();
        assert!(matches!(err, MessageError::MissingResultOrError { id: 9 }));
        assert!(err.to_string().contains('9'));
    }

    #[test]
    fn rejects_malformed_error_body() {
        let err = classify(br#"{"id": 2, "error": "boom"}"#).unwrap_err();
        assert!(matches!(err, MessageError::Payload { .. }));
    }

    #[test]
    fn error_body_with_extra_or_missing_keys_is_tolerated() {
        let inbound = classify(br#"{"id": 2, "error": {"code": 7}}"#).unwrap();
        assert_eq!(
            inbound,
            Inbound::Error {
                id: 2,
                error: ApiError {
                    error: None,
                    message: None,
                },
            }
        );
    }
}
