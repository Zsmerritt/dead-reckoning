//! In-process fake Moonraker WebSocket server for tests.
//!
//! Loopback only, by policy: executor tests must NEVER touch a real
//! printer or a real Moonraker instance (one exists on this network).
//! The fake binds `127.0.0.1:0`, speaks just enough JSON-RPC 2.0 for
//! the client under test, records every call, and pushes a gratuitous
//! notification before each response so clients prove they skip them.
//!
//! ## Why the fake runs on its own dedicated OS thread
//!
//! Every test that drives a [`FakeMoonraker`] shares the process with
//! hundreds of sibling tests. Under full-suite parallelism the OS badly
//! oversubscribes CPUs (cargo alone runs tests on as many threads as
//! there are cores, and several of *those* tests each spin up their own
//! multi-worker tokio runtime on top). A fake that merely `tokio::spawn`s
//! its accept/serve loop onto whatever runtime happens to be driving the
//! calling test shares that runtime's worker threads — and therefore its
//! OS-level scheduling fate — with everything else the test does. When a
//! worker thread carrying the fake's accept loop gets starved for long
//! enough, the fake stops responding and any test whose timing assumed a
//! prompt reply goes flaky, even though the code under test is fine.
//!
//! [`FakeMoonraker::spawn`] therefore binds the loopback listener
//! synchronously (an instant syscall, so the bound address is known
//! before any thread exists) and then hands the whole accept/serve loop
//! to a brand-new OS thread running its own single-threaded tokio
//! runtime. That thread does nothing else — it is never a party to
//! whatever scheduling pressure the rest of the suite puts on the
//! runtime under test — so suite-wide executor contention cannot stall
//! it. This generalizes a pattern one test (`a_held_gcode_mutex_times_out
//! _into_a_refusal` in `recover.rs`) already used by hand for a related
//! reason (a blocking handler must not stall the runtime under test);
//! every call site gets the isolation for free.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};

use serde_json::{json, Value};
use tokio_tungstenite::tungstenite::Message;

/// Handler: `(method, params)` → `Ok(result)` or `Err((code, message))`.
pub type Handler = dyn Fn(&str, &Value) -> Result<Value, (i64, String)> + Send + Sync;

/// A running fake Moonraker. Its accept/serve loop lives on a dedicated
/// OS thread with its own tokio runtime — see the module docs.
pub struct FakeMoonraker {
    addr: SocketAddr,
    calls: Arc<Mutex<Vec<(String, Value)>>>,
    /// Dropping the sender tells the dedicated thread's accept loop to
    /// stop; `None` once shutdown has been signalled.
    shutdown: Option<tokio::sync::oneshot::Sender<()>>,
    /// Joined on drop so the OS thread (and every connection it is
    /// still serving) is torn down before `Drop::drop` returns, instead
    /// of leaking a thread per test.
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Drop for FakeMoonraker {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            // Best-effort: if the thread already exited, the receiver
            // is gone and there is nothing left to signal.
            let _ = shutdown.send(());
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl FakeMoonraker {
    /// Binds a loopback listener and serves `handler` on every
    /// connection (sequential reconnects supported), on a dedicated OS
    /// thread isolated from whatever runtime the caller is on.
    pub async fn spawn(
        handler: impl Fn(&str, &Value) -> Result<Value, (i64, String)> + Send + Sync + 'static,
    ) -> Self {
        // A plain std bind: instant, and it lets us hand the listener to
        // the dedicated thread below without that thread needing to be
        // the one to pick the port.
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind loopback");
        listener
            .set_nonblocking(true)
            .expect("set loopback listener nonblocking");
        let addr = listener.local_addr().expect("local addr");
        let calls: Arc<Mutex<Vec<(String, Value)>>> = Arc::new(Mutex::new(Vec::new()));
        let handler: Arc<Handler> = Arc::new(handler);
        let calls_task = Arc::clone(&calls);
        let (shutdown_tx, mut shutdown_rx) = tokio::sync::oneshot::channel::<()>();
        // Signalled once the listener is registered with the dedicated
        // thread's own reactor, so `spawn` cannot return (and a caller
        // cannot race a connection attempt) before the fake is actually
        // able to accept.
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel::<()>();

        let thread = std::thread::Builder::new()
            .name("fake-moonraker".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("fake moonraker runtime");
                rt.block_on(async move {
                    // Registers the std listener with *this* runtime's
                    // reactor; must happen inside the runtime it will be
                    // polled on.
                    let listener = tokio::net::TcpListener::from_std(listener)
                        .expect("register loopback listener");
                    let _ = ready_tx.send(());
                    loop {
                        tokio::select! {
                            accepted = listener.accept() => {
                                let Ok((stream, _)) = accepted else {
                                    return;
                                };
                                let handler = Arc::clone(&handler);
                                let calls = Arc::clone(&calls_task);
                                tokio::spawn(async move {
                                    let _ = serve_connection(stream, &handler, &calls).await;
                                });
                            }
                            _ = &mut shutdown_rx => {
                                return;
                            }
                        }
                    }
                });
            })
            .expect("spawn fake moonraker thread");

        // If the thread died before registering the listener, the sender
        // was dropped; `expect` surfaces that as a clear test failure
        // instead of a silent race against an accept loop that never
        // starts.
        ready_rx.await.expect("fake moonraker thread came up");

        Self {
            addr,
            calls,
            shutdown: Some(shutdown_tx),
            thread: Some(thread),
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
