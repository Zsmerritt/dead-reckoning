//! Typed construction of Klipper API socket requests.
//!
//! Requests are JSON objects `{"id": <int>, "method": <endpoint>,
//! "params": <dict>}` terminated by `0x03` (`klippy/webhooks.py`,
//! `WebRequest.__init__` and `ClientConnection.process_received`;
//! `docs/API_Server.md`). The `id` is chosen by the client and echoed
//! verbatim in the matching response; `params` must be a JSON object.
//!
//! Subscription-style endpoints accept a `response_template` dictionary
//! whose keys are echoed at the top level of every later asynchronous
//! message (`docs/API_Server.md`, "Subscriptions"). Give each subscription
//! a distinct template key so the daemon can demultiplex notifications.

use std::collections::BTreeMap;

use serde_json::{Map, Value};

use crate::error::EncodeError;
use crate::frame::ETX;

/// Object selection for `objects/subscribe` and `objects/query`: object
/// name to requested field names, where `None` requests every field
/// (`klippy/webhooks.py`, `QueryStatusHelper._handle_query` accepts a
/// `null` field list and expands it to all fields). A `BTreeMap` keeps the
/// serialized request deterministic.
pub type SubscriptionObjects = BTreeMap<String, Option<Vec<String>>>;

/// A response template: arbitrary key/value pairs echoed at the top level
/// of each asynchronous message produced by a subscription.
pub type ResponseTemplate = Map<String, Value>;

/// A typed Klipper API request.
///
/// Build a variant, then serialize it with [`Request::to_frame`].
#[derive(Debug, Clone, PartialEq)]
pub enum Request {
    /// `info` — printer state, paths and version
    /// (`klippy/webhooks.py`, `WebHooks._handle_info_request`).
    Info {
        /// Optional client identification recorded in Klipper's log.
        client_info: Option<Map<String, Value>>,
    },
    /// `objects/subscribe` — query and then subscribe to printer object
    /// status (`klippy/webhooks.py`, `QueryStatusHelper._handle_subscribe`).
    ObjectsSubscribe {
        /// Objects and fields to subscribe to.
        objects: SubscriptionObjects,
        /// Template echoed in each status update message.
        response_template: Option<ResponseTemplate>,
    },
    /// `objects/query` — one-shot printer object status query
    /// (`klippy/webhooks.py`, `QueryStatusHelper._handle_query`).
    ObjectsQuery {
        /// Objects and fields to query.
        objects: SubscriptionObjects,
    },
    /// `motion_report/dump_trapq` — subscribe to a trapezoidal motion
    /// queue (`klippy/extras/motion_report.py`, `DumpTrapQ`; mux endpoint
    /// keyed by `name`).
    DumpTrapq {
        /// Trapq name: `"toolhead"`, `"extruder"`, `"extruder1"`, ...
        name: String,
        /// Template echoed in each batch message.
        response_template: Option<ResponseTemplate>,
    },
    /// `motion_report/dump_stepper` — subscribe to a stepper's
    /// `queue_step` stream (`klippy/extras/motion_report.py`,
    /// `DumpStepper`; mux endpoint keyed by `name`).
    DumpStepper {
        /// Stepper name, e.g. `"stepper_z"`.
        name: String,
        /// Template echoed in each batch message.
        response_template: Option<ResponseTemplate>,
    },
    /// `gcode/script` — run a G-Code script; the response is sent when the
    /// script completes (`klippy/webhooks.py`, `GCodeHelper._handle_script`).
    GcodeScript {
        /// The G-Code script to execute.
        script: String,
    },
    /// `gcode/subscribe_output` — subscribe to G-Code terminal output
    /// (`klippy/webhooks.py`, `GCodeHelper._handle_subscribe_output`).
    GcodeSubscribeOutput {
        /// Template echoed in each output message.
        response_template: Option<ResponseTemplate>,
    },
}

impl Request {
    /// The endpoint name sent in the `method` field.
    #[must_use]
    pub fn method(&self) -> &'static str {
        match self {
            Request::Info { .. } => "info",
            Request::ObjectsSubscribe { .. } => "objects/subscribe",
            Request::ObjectsQuery { .. } => "objects/query",
            Request::DumpTrapq { .. } => "motion_report/dump_trapq",
            Request::DumpStepper { .. } => "motion_report/dump_stepper",
            Request::GcodeScript { .. } => "gcode/script",
            Request::GcodeSubscribeOutput { .. } => "gcode/subscribe_output",
        }
    }

    /// The `params` object for this request.
    pub fn params(&self) -> Result<Map<String, Value>, EncodeError> {
        let mut params = Map::new();
        match self {
            Request::Info { client_info } => {
                if let Some(ci) = client_info {
                    params.insert("client_info".to_owned(), Value::Object(ci.clone()));
                }
            }
            Request::ObjectsSubscribe {
                objects,
                response_template,
            } => {
                params.insert("objects".to_owned(), serde_json::to_value(objects)?);
                insert_template(&mut params, response_template.as_ref());
            }
            Request::ObjectsQuery { objects } => {
                params.insert("objects".to_owned(), serde_json::to_value(objects)?);
            }
            Request::DumpTrapq {
                name,
                response_template,
            }
            | Request::DumpStepper {
                name,
                response_template,
            } => {
                params.insert("name".to_owned(), Value::String(name.clone()));
                insert_template(&mut params, response_template.as_ref());
            }
            Request::GcodeScript { script } => {
                params.insert("script".to_owned(), Value::String(script.clone()));
            }
            Request::GcodeSubscribeOutput { response_template } => {
                insert_template(&mut params, response_template.as_ref());
            }
        }
        Ok(params)
    }

    /// Serializes the request with the given client-chosen `id` to wire
    /// bytes, including the trailing `0x03` terminator.
    pub fn to_frame(&self, id: u64) -> Result<Vec<u8>, EncodeError> {
        let mut msg = Map::new();
        msg.insert("id".to_owned(), Value::from(id));
        msg.insert("method".to_owned(), Value::String(self.method().to_owned()));
        msg.insert("params".to_owned(), Value::Object(self.params()?));
        let mut bytes = serde_json::to_vec(&Value::Object(msg))?;
        bytes.push(ETX);
        Ok(bytes)
    }
}

fn insert_template(params: &mut Map<String, Value>, template: Option<&ResponseTemplate>) {
    if let Some(t) = template {
        params.insert("response_template".to_owned(), Value::Object(t.clone()));
    }
}

#[cfg(test)]
mod tests {
    use super::{Request, ResponseTemplate, SubscriptionObjects};
    use crate::frame::ETX;
    use serde_json::{json, Value};

    /// Parses the wire bytes back to JSON after checking the terminator.
    fn decode(frame: &[u8]) -> Value {
        assert_eq!(*frame.last().unwrap(), ETX);
        assert!(
            !frame[..frame.len() - 1].contains(&ETX),
            "ETX must only terminate the frame"
        );
        serde_json::from_slice(&frame[..frame.len() - 1]).unwrap()
    }

    #[test]
    fn info_request_minimal() {
        let frame = Request::Info { client_info: None }.to_frame(1).unwrap();
        assert_eq!(
            decode(&frame),
            json!({"id": 1, "method": "info", "params": {}})
        );
    }

    #[test]
    fn info_request_with_client_info() {
        let ci = json!({"program": "plrd", "version": "0.1.0"});
        let Value::Object(ci_map) = ci.clone() else {
            unreachable!()
        };
        let frame = Request::Info {
            client_info: Some(ci_map),
        }
        .to_frame(7)
        .unwrap();
        assert_eq!(
            decode(&frame),
            json!({"id": 7, "method": "info", "params": {"client_info": ci}})
        );
    }

    #[test]
    fn objects_subscribe_request_matches_api_server_doc() {
        // Shape per docs/API_Server.md "objects/subscribe" example.
        let mut objects = SubscriptionObjects::new();
        objects.insert(
            "toolhead".to_owned(),
            Some(vec!["position".to_owned(), "print_time".to_owned()]),
        );
        objects.insert("gcode_move".to_owned(), None);
        let mut template = ResponseTemplate::new();
        template.insert("q".to_owned(), json!("status"));
        let frame = Request::ObjectsSubscribe {
            objects,
            response_template: Some(template),
        }
        .to_frame(123)
        .unwrap();
        assert_eq!(
            decode(&frame),
            json!({
                "id": 123,
                "method": "objects/subscribe",
                "params": {
                    // null selects every field (webhooks.py _handle_query).
                    "objects": {"gcode_move": null,
                                "toolhead": ["position", "print_time"]},
                    "response_template": {"q": "status"},
                }
            })
        );
    }

    #[test]
    fn objects_query_request() {
        let mut objects = SubscriptionObjects::new();
        objects.insert("virtual_sdcard".to_owned(), None);
        let frame = Request::ObjectsQuery { objects }.to_frame(2).unwrap();
        assert_eq!(
            decode(&frame),
            json!({
                "id": 2,
                "method": "objects/query",
                "params": {"objects": {"virtual_sdcard": null}}
            })
        );
    }

    #[test]
    fn dump_trapq_request_matches_api_server_doc() {
        // Shape per docs/API_Server.md "motion_report/dump_trapq" example.
        let frame = Request::DumpTrapq {
            name: "toolhead".to_owned(),
            response_template: None,
        }
        .to_frame(123)
        .unwrap();
        assert_eq!(
            decode(&frame),
            json!({
                "id": 123,
                "method": "motion_report/dump_trapq",
                "params": {"name": "toolhead"}
            })
        );
    }

    #[test]
    fn dump_stepper_request_with_template() {
        let mut template = ResponseTemplate::new();
        template.insert("q".to_owned(), json!("stepper_z"));
        let frame = Request::DumpStepper {
            name: "stepper_z".to_owned(),
            response_template: Some(template),
        }
        .to_frame(5)
        .unwrap();
        assert_eq!(
            decode(&frame),
            json!({
                "id": 5,
                "method": "motion_report/dump_stepper",
                "params": {"name": "stepper_z",
                           "response_template": {"q": "stepper_z"}}
            })
        );
    }

    #[test]
    fn gcode_script_request() {
        let frame = Request::GcodeScript {
            script: "M117 hello \u{3}".to_owned(),
        }
        .to_frame(9)
        .unwrap();
        // JSON escapes the control character, so the ETX cannot appear raw.
        assert_eq!(
            decode(&frame),
            json!({
                "id": 9,
                "method": "gcode/script",
                "params": {"script": "M117 hello \u{3}"}
            })
        );
    }

    #[test]
    fn gcode_subscribe_output_matches_api_server_doc() {
        // Shape per docs/API_Server.md "gcode/subscribe_output" example.
        let mut template = ResponseTemplate::new();
        template.insert("key".to_owned(), json!(345));
        let frame = Request::GcodeSubscribeOutput {
            response_template: Some(template),
        }
        .to_frame(123)
        .unwrap();
        assert_eq!(
            decode(&frame),
            json!({
                "id": 123,
                "method": "gcode/subscribe_output",
                "params": {"response_template": {"key": 345}}
            })
        );
    }

    #[test]
    fn u64_max_id_round_trips() {
        let frame = Request::Info { client_info: None }
            .to_frame(u64::MAX)
            .unwrap();
        assert_eq!(decode(&frame)["id"], json!(u64::MAX));
    }
}
