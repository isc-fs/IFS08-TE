//! Session layer — handshake, keepalive, reconnect, notification
//! streams.
//!
//! The pieces below the [`Session`] type (`transport::CanBackend`,
//! `protocol::IsoTp*`, `protocol::Response`) are deliberately low-level:
//! one call per frame, no retry logic, no understanding of
//! "connected" vs "not connected". Real subcommands don't want to
//! drive that machinery directly — they'd reinvent the same
//! boilerplate (segment → send → reassemble → parse → check NACK →
//! refresh keepalive) every time. This module collapses all of that
//! into one type.
//!
//! ## Concurrency model
//!
//! ```text
//! ┌──────────────┐     backend.recv()     ┌──────────────┐
//! │   rx_task    │ ◄──────────────────── │   backend    │
//! │ (background) │                        └──────┬───────┘
//! │              │                               │ backend.send()
//! │   routes:    │                               ▲
//! │   ACK/NACK/  │──► reply_tx (mpsc) ──┐       │
//! │   DISCOVER   │                       │       │
//! │   NOTIFY ───►│ notification_tx       │       │
//! │              │    (broadcast)        │       │
//! └──────────────┘                       ▼       │
//!                                ┌──────────────┐│
//!                                │ send_command │├─ command_lock
//!                                │ broadcast()  │◄ serialises
//!                                │ connect()    ││ all ops
//!                                └──────────────┘│
//! ```
//!
//! - Exactly one task calls `backend.recv()`: the RX task. Everyone
//!   else consumes pre-decoded [`Response`]s from the reply channel
//!   (single-receiver mpsc — only the current command is listening)
//!   or the notification channel (broadcast — any number of
//!   subscribers).
//! - Exactly one command is in flight at a time. `command_lock`
//!   serialises `send_command`, `broadcast`, `connect` and
//!   `disconnect` so a concurrent "retry on BAD_SESSION" doesn't
//!   race with another op.
//!
//! ## What Session does NOT do
//!
//! - **Protocol encoding.** Callers build their own payload bytes via
//!   [`crate::protocol::commands`] and pass them in. Session just
//!   segments, sends, reassembles, and parses the reply — it has no
//!   opinion on what's in the payload.
//! - **Interpretation of Notify payloads.** The subscriber gets raw
//!   [`Response::Notify`]; decoding to a `NotifyOpcode` + struct is
//!   the subscriber's job (matches how the rest of the protocol
//!   module works).
//! - **Flash orchestration.** Sector-aware erase / diff / CRC
//!   verification live in `src/firmware/` (feat/12+).
//!
//! ## Reliability invariant — **no unbounded `.await` in this module**
//!
//! Every `.await` point is either wrapped in a `tokio::time::timeout`
//! with a finite deadline, or provably bounded by construction (a
//! sleep, a mutex with no reentrancy, a channel whose sender is
//! held by a task that will complete). The RX daemon's outer `loop`
//! is unbounded by design, but every await inside it has a 50 ms
//! read deadline so shutdown is prompt.
//!
//! This is the invariant that three separate post-mortems (fix/10,
//! fix/14, and the fix/15 audit) converged on — a missed deadline
//! turned into a 25-minute silent hang or a "pkill-only" recovery.
//! **Before adding a new `.await` to this module, justify its
//! boundedness in a comment right at the await site** using one of:
//!
//! - `// bounded by `X` ms via `tokio::time::timeout`
//! - `// bounded: tokio mutex, only held across non-awaiting code
//! - `// bounded: channel drained by <task>, which always completes
//! - `// by design: daemon outer loop / `ctrl_c` arm in `select!`
//!
//! A reviewer must push back on awaits without such a comment. The
//! cost of regressing here is hard to detect (symptom is a hang
//! indistinguishable from a long-running operation), so the bar at
//! review time is higher than "does it compile."

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::{broadcast, mpsc, oneshot, Mutex as TokioMutex};
use tokio::task::JoinHandle;
use tokio::time::timeout;
use tracing::{debug, trace, warn};

use crate::app_control::{reboot_to_bl_payload, BootloaderEntry, REBOOT_TO_BL_ID};
use crate::protocol::commands::{
    cmd_connect, cmd_disconnect, cmd_get_health, PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR,
};
use crate::protocol::ids::{FrameId, MessageType};
use crate::protocol::isotp::{IsoTpSegmenter, ReassembleOutcome, Reassembler};
use crate::protocol::opcodes::{CommandOpcode, NackCode};
use crate::protocol::{CanFrame, Response, BROADCAST_NODE_ID};
use crate::transport::{CanBackend, TransportError};

/// Linear-backoff base between retries in [`Session::send_command_retrying`]
/// (FMEA #271 G11). Attempt N waits `N * this` before re-sending —
/// short enough not to stall a healthy flash, long enough to ride out
/// a transient adapter TX-buffer hiccup.
const COMMAND_RETRY_BACKOFF_MS: u64 = 50;

/// Everything this layer can fail with. Wraps the lower-level
/// `TransportError` / `ParseError` variants and adds session-specific
/// conditions (NACK, protocol-version mismatch, not-connected, …).
#[derive(Debug, thiserror::Error)]
pub enum SessionError {
    #[error(transparent)]
    Transport(#[from] TransportError),

    #[error(transparent)]
    Parse(#[from] crate::protocol::ParseError),

    /// Device NACK'd the command. `rejected_opcode` is the opcode
    /// the device says it rejected (`0xFF` when the device didn't
    /// successfully identify what we sent).
    #[error("device NACK'd opcode 0x{rejected_opcode:02X} with code {code}")]
    Nack { rejected_opcode: u8, code: NackCode },

    /// Peer answered CONNECT with `NACK(PROTOCOL_VERSION)` or a
    /// version number we don't support.
    #[error("protocol version mismatch: host {host_major}.{host_minor}, device {device_major}.{device_minor}")]
    ProtocolVersionMismatch {
        host_major: u8,
        host_minor: u8,
        device_major: u8,
        device_minor: u8,
    },

    /// No reply arrived within the configured command timeout.
    ///
    /// `adapter_errors_during_wait` is the number of adapter-level
    /// error events the backend saw while this command was in
    /// flight — on SLCAN that's BEL (`0x07`) bytes from the
    /// CANable. A non-zero value means the adapter refused our TX
    /// frame (bus-off, TX buffer full, stuck-dominant), so the
    /// frame never reached the CAN bus and the device was never
    /// going to reply. We print the count in the error so
    /// operators know to unplug/replug the CANable rather than
    /// assume the target is dead.
    #[error(
        "{}",
        command_timeout_display(*.timeout, *.adapter_errors_during_wait)
    )]
    CommandTimeout {
        timeout: Duration,
        adapter_errors_during_wait: u32,
    },

    /// Session operation attempted without a prior successful
    /// `connect()`.
    #[error("session-gated operation attempted before connect()")]
    NotConnected,

    /// RX task has exited — the backend is closed or the session has
    /// been dropped.
    #[error("session RX task exited — backend may have disconnected")]
    RxClosed,

    /// Shouldn't reach this path; RX / keepalive task panicked.
    #[error("session task panic: {0}")]
    TaskPanic(String),

    /// Received a `CMD` frame at the host, a duplicate FF mid-
    /// reassembly, or anything else the protocol layer classifies as
    /// "garbled bus traffic".
    #[error("unexpected protocol frame: {0}")]
    Protocol(&'static str),
}

/// Render the `CommandTimeout` error message. Split into a helper so
/// `thiserror`'s `#[error(...)]` can invoke it cleanly via
/// `{}` + a function pointer, and so the string format is covered by
/// its own unit tests (in `transport::slcan::tests`) without going
/// through the whole error chain. `pub(crate)` so those tests can
/// reach it; external callers should rely on the `Display` impl of
/// `SessionError` instead.
pub(crate) fn command_timeout_display(timeout: Duration, adapter_errors: u32) -> String {
    if adapter_errors == 0 {
        format!(
            "timed out waiting for device reply after {}ms",
            timeout.as_millis()
        )
    } else {
        format!(
            "timed out waiting for device reply after {}ms; adapter reported {} error(s) \
             during this wait — the frame probably never reached the CAN bus (bus-off, \
             TX buffer full, or stuck-dominant). Try unplugging and replugging the \
             adapter, then retry.",
            timeout.as_millis(),
            adapter_errors,
        )
    }
}

/// Per-session knobs. [`SessionConfig::default`] gives you numbers
/// that match REQUIREMENTS.md — `target_node: 0x3`, 5 s keepalive,
/// 500 ms command timeout.
#[derive(Debug, Clone)]
pub struct SessionConfig {
    pub target_node: u8,
    pub keepalive_interval: Duration,
    pub command_timeout: Duration,
    pub host_major: u8,
    pub host_minor: u8,
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self {
            target_node: 0x3,
            keepalive_interval: Duration::from_millis(5_000),
            command_timeout: Duration::from_millis(500),
            host_major: PROTOCOL_VERSION_MAJOR,
            host_minor: PROTOCOL_VERSION_MINOR,
        }
    }
}

/// Per-attempt `CMD_CONNECT` timeout while entering the bootloader.
/// When a bootloader is present it ACKs within a few ms, so we keep this
/// short — deliberately decoupled from the (erase-sized) session
/// `command_timeout`, which can be seconds — so the reboot-to-BL poll
/// loop probes many times inside the window instead of blocking whole
/// seconds on each miss.
const BL_CONNECT_ATTEMPT_TIMEOUT: Duration = Duration::from_millis(750);

/// Delay between successive CONNECT probes while polling a target into
/// the bootloader. One probe cycle is roughly this plus up to one
/// [`BL_CONNECT_ATTEMPT_TIMEOUT`].
const BL_REBOOT_POLL_INTERVAL: Duration = Duration::from_millis(200);

/// Minimum gap between reboot-to-BL triggers while polling. Deliberately
/// several probe cycles long: re-sending the trigger too aggressively
/// can keep restarting a board's reset → bootloader-boot sequence so it
/// never settles long enough to answer CONNECT (observed on the ECU —
/// one trigger reaches a stable bootloader, rapid re-triggering never
/// does). We still re-send periodically to cover a dropped trigger.
const BL_REBOOT_RETRIGGER_INTERVAL: Duration = Duration::from_millis(2_500);

/// The handle a caller holds. Internally shares state with the RX
/// task + optional keepalive task via `Arc<SessionInner>`.
pub struct Session {
    inner: Arc<SessionInner>,
    rx_handle: TokioMutex<Option<JoinHandle<()>>>,
    rx_shutdown: Arc<AtomicBool>,
    keepalive: TokioMutex<Option<KeepaliveState>>,
    config: SessionConfig,
    /// Latched at the end of a successful `connect()`. Cleared by
    /// `disconnect()`. `send_session_gated` checks it before
    /// attempting a command.
    connected: AtomicBool,
}

struct SessionInner {
    backend: Arc<dyn CanBackend>,
    target_node: u8,
    /// Single-receiver channel for ACK / NACK / DISCOVER replies.
    /// The RX task holds the sender; the current command holder
    /// (whoever acquired `command_lock`) owns the receiver via a
    /// Mutex.
    reply_rx: TokioMutex<mpsc::Receiver<Response>>,
    /// Broadcast channel for unsolicited NOTIFYs (HEARTBEAT, DTC,
    /// LOG, LIVE_DATA). Subscribers call `session.subscribe_notifications()`.
    notification_tx: broadcast::Sender<Response>,
    /// Serialises send_command / broadcast / connect / disconnect so
    /// only one command is ever in flight. The `u8` is unused;
    /// exists to make the Mutex non-ZST (easier to print / debug).
    command_lock: TokioMutex<()>,
}

struct KeepaliveState {
    cancel_tx: oneshot::Sender<()>,
    handle: JoinHandle<()>,
}

impl Session {
    /// Attach a session over an already-opened backend. The RX task
    /// starts immediately; call [`Session::connect`] before issuing
    /// any session-gated command.
    ///
    /// Session-less commands (DISCOVER, GET_FW_INFO, GET_HEALTH,
    /// OB_READ, DTC_READ, RESET, JUMP) work without `connect()` —
    /// just `send_command`.
    pub fn attach(backend: Box<dyn CanBackend>, config: SessionConfig) -> Self {
        let backend_arc: Arc<dyn CanBackend> = Arc::from(backend);
        let (reply_tx, reply_rx) = mpsc::channel::<Response>(32);
        // 64-deep broadcast is plenty: the only reason a subscriber
        // falls behind is they're not consuming. Dropped messages
        // surface as `RecvError::Lagged`.
        let (notification_tx, _) = broadcast::channel::<Response>(64);

        let inner = Arc::new(SessionInner {
            backend: Arc::clone(&backend_arc),
            target_node: config.target_node,
            reply_rx: TokioMutex::new(reply_rx),
            notification_tx: notification_tx.clone(),
            command_lock: TokioMutex::new(()),
        });

        let shutdown = Arc::new(AtomicBool::new(false));
        let rx_handle = tokio::spawn(rx_task(
            backend_arc,
            reply_tx,
            notification_tx,
            config.target_node,
            Arc::clone(&shutdown),
        ));

        Self {
            inner,
            rx_handle: TokioMutex::new(Some(rx_handle)),
            rx_shutdown: shutdown,
            keepalive: TokioMutex::new(None),
            config,
            connected: AtomicBool::new(false),
        }
    }

    /// True after a successful `connect()` and until `disconnect()`
    /// (or the watchdog / bus drops us).
    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    /// Perform the `CMD_CONNECT` handshake. Sends `[major, minor]`,
    /// waits for an ACK carrying the device's advertised version,
    /// validates majors match, starts the keepalive task.
    pub async fn connect(&self) -> Result<(u8, u8), SessionError> {
        self.connect_with_timeout(self.config.command_timeout).await
    }

    /// [`connect`](Self::connect), but waits only `reply_timeout` for the
    /// CONNECT ACK instead of the session `command_timeout`. Used by the
    /// bootloader-entry poll loop, which wants fast per-attempt probes.
    async fn connect_with_timeout(
        &self,
        reply_timeout: Duration,
    ) -> Result<(u8, u8), SessionError> {
        let _guard = self.inner.command_lock.lock().await;
        let payload = cmd_connect(self.config.host_major, self.config.host_minor);
        let response = self
            .send_raw_with_timeout(&payload, MessageType::Cmd, reply_timeout)
            .await?;

        match response {
            Response::Ack { opcode, payload } => {
                if opcode != CommandOpcode::Connect.as_byte() {
                    return Err(SessionError::Protocol(
                        "CONNECT ACK has unexpected opcode byte",
                    ));
                }
                if payload.len() < 2 {
                    return Err(SessionError::Protocol(
                        "CONNECT ACK payload shorter than [major, minor]",
                    ));
                }
                let device_major = payload[0];
                let device_minor = payload[1];
                if device_major != self.config.host_major {
                    return Err(SessionError::ProtocolVersionMismatch {
                        host_major: self.config.host_major,
                        host_minor: self.config.host_minor,
                        device_major,
                        device_minor,
                    });
                }

                self.connected.store(true, Ordering::SeqCst);
                self.start_keepalive().await;
                debug!(device_major, device_minor, "session connected");
                Ok((device_major, device_minor))
            }
            Response::Nack {
                rejected_opcode,
                code,
            } => {
                if code == NackCode::ProtocolVersion {
                    // We don't know the device's version for logging
                    // here; return a best-effort mismatch error.
                    Err(SessionError::ProtocolVersionMismatch {
                        host_major: self.config.host_major,
                        host_minor: self.config.host_minor,
                        device_major: 0,
                        device_minor: 0,
                    })
                } else {
                    Err(SessionError::Nack {
                        rejected_opcode,
                        code,
                    })
                }
            }
            other => Err(SessionError::Protocol(match other {
                Response::Notify { .. } => "unexpected NOTIFY during CONNECT",
                Response::Discover { .. } => "unexpected DISCOVER during CONNECT",
                _ => "unexpected response during CONNECT",
            })),
        }
    }

    /// Send one raw classic-CAN frame, bypassing the bootloader
    /// protocol entirely (no ISO-TP, no node-id encoding). For
    /// app-level signals a *running application* listens for — e.g.
    /// the reboot-to-bootloader trigger. The session need not be
    /// connected (the frame goes straight out on the backend).
    pub async fn send_app_frame(&self, id: u16, data: &[u8]) -> Result<(), SessionError> {
        let frame = CanFrame::new(id, data)?;
        self.inner.backend.send(frame).await?;
        Ok(())
    }

    /// CONNECT, transparently rebooting a *running application* into
    /// the bootloader when needed (see [`BootloaderEntry`]).
    ///
    /// `window` bounds how long we keep trying to reach the bootloader
    /// after the reboot trigger. Rather than a single fixed sleep + one
    /// retry — fragile against reboot-timing jitter, a dropped trigger,
    /// or a bootloader whose post-reset listen window is short — we
    /// *poll*: re-send the trigger and probe CONNECT (with a short
    /// per-attempt timeout) every round until the bootloader answers or
    /// `window` elapses.
    ///
    /// - `Never`: plain [`connect`](Self::connect), no trigger.
    /// - `Auto`: one short CONNECT probe first (fast path for a board
    ///   already in the bootloader); on timeout, fall into the reboot
    ///   poll loop.
    /// - `Always`: straight into the reboot poll loop.
    pub async fn connect_entering_bootloader(
        &self,
        entry: BootloaderEntry,
        window: Duration,
    ) -> Result<(u8, u8), SessionError> {
        match entry {
            BootloaderEntry::Never => self.connect().await,
            BootloaderEntry::Always => self.reboot_and_connect(window).await,
            BootloaderEntry::Auto => {
                match self.connect_with_timeout(BL_CONNECT_ATTEMPT_TIMEOUT).await {
                    Ok(v) => Ok(v),
                    Err(SessionError::CommandTimeout { .. }) => {
                        self.reboot_and_connect(window).await
                    }
                    Err(e) => Err(e),
                }
            }
        }
    }

    /// Poll a running application into the bootloader: send the
    /// reboot-to-BL trigger, then probe CONNECT with a short timeout and
    /// keep probing — re-sending the trigger only every
    /// [`BL_REBOOT_RETRIGGER_INTERVAL`], not every probe — until the
    /// bootloader answers or `window` elapses.
    ///
    /// The trigger cadence matters: a board's reset → bootloader-boot
    /// sequence (open HV relays, drain, `NVIC_SystemReset`, BL preamble)
    /// takes a beat, and re-triggering *during* it can restart the whole
    /// sequence so the board never settles long enough to answer. So we
    /// give it an uninterrupted stretch after each trigger, probing
    /// frequently, and only re-send periodically to cover a genuinely
    /// dropped trigger.
    async fn reboot_and_connect(&self, window: Duration) -> Result<(u8, u8), SessionError> {
        let deadline = Instant::now() + window;
        let mut attempts: u32 = 0;
        loop {
            self.send_app_frame(
                REBOOT_TO_BL_ID,
                &reboot_to_bl_payload(self.config.target_node),
            )
            .await?;
            let next_trigger = Instant::now() + BL_REBOOT_RETRIGGER_INTERVAL;

            // Probe CONNECT until it's time to re-send the trigger — or
            // the overall deadline passes.
            loop {
                tokio::time::sleep(BL_REBOOT_POLL_INTERVAL).await;
                attempts += 1;
                match self.connect_with_timeout(BL_CONNECT_ATTEMPT_TIMEOUT).await {
                    Ok(v) => {
                        debug!(attempts, "bootloader answered after reboot trigger");
                        return Ok(v);
                    }
                    Err(e @ SessionError::CommandTimeout { .. }) => {
                        let now = Instant::now();
                        if now >= deadline {
                            warn!(
                                attempts,
                                "bootloader did not answer within the reboot window"
                            );
                            return Err(e);
                        }
                        if now >= next_trigger {
                            break; // re-send the trigger, then keep probing
                        }
                        // else: keep probing without re-triggering
                    }
                    Err(e) => return Err(e),
                }
            }
        }
    }

    /// Send a single command and wait for its ACK / NACK.
    /// ISO-TP-framed on both sides. `message_type` is almost always
    /// `MessageType::Cmd`; DISCOVER callers use [`Session::broadcast`]
    /// instead.
    pub async fn send_command(&self, payload: &[u8]) -> Result<Response, SessionError> {
        let _guard = self.inner.command_lock.lock().await;
        self.send_raw(payload, MessageType::Cmd).await
    }

    /// Like [`Session::send_command`] but sends to `dst` instead of
    /// the session's configured `target_node`. Used by `discover`:
    /// after a broadcast collects replies from multiple nodes, each
    /// responder gets follow-up `GET_FW_INFO` / `GET_HEALTH` pings
    /// routed individually without reattaching the session.
    ///
    /// The broader session state (keepalive, reconnect-on-BAD_SESSION,
    /// notification routing) is unaffected — this is purely a
    /// per-call destination override.
    ///
    /// NOTE (FMEA #271 G7): the rx task only forwards command replies
    /// whose source matches the session's `target_node`, *unless* the
    /// session targets `BROADCAST_NODE_ID`. So `send_command_to` is
    /// meant for **broadcast-target sessions** (which is exactly how
    /// `discover` uses it). Calling it on a session pinned to a single
    /// node, with a `dst` other than that node, would see the reply
    /// filtered out.
    pub async fn send_command_to(&self, dst: u8, payload: &[u8]) -> Result<Response, SessionError> {
        let _guard = self.inner.command_lock.lock().await;
        let errors_before = self.inner.backend.adapter_error_count();
        self.send_frames(payload, MessageType::Cmd, dst).await?;
        let mut rx = self.inner.reply_rx.lock().await;
        match timeout(self.config.command_timeout, rx.recv()).await {
            Ok(Some(response)) => Ok(response),
            Ok(None) => Err(SessionError::RxClosed),
            Err(_) => Err(SessionError::CommandTimeout {
                timeout: self.config.command_timeout,
                adapter_errors_during_wait: self
                    .inner
                    .backend
                    .adapter_error_count()
                    .saturating_sub(errors_before),
            }),
        }
    }

    /// Like [`Session::send_command`], but for session-gated opcodes:
    /// on `NACK(BAD_SESSION)`, reconnect and retry the command once.
    /// If the retry still fails (or the reconnect fails) the error
    /// bubbles up.
    pub async fn send_session_gated(&self, payload: &[u8]) -> Result<Response, SessionError> {
        // Grab the command lock for the whole "try → reconnect → retry"
        // sequence so another caller can't interleave mid-reconnect.
        let _guard = self.inner.command_lock.lock().await;
        let first = self.send_raw(payload, MessageType::Cmd).await?;
        match &first {
            Response::Nack { code, .. } if *code == NackCode::BadSession => {
                debug!("received BAD_SESSION — reconnecting + retrying");
                self.connected.store(false, Ordering::SeqCst);
                self.stop_keepalive_locked().await;
                // Inline CONNECT — we already hold command_lock.
                let connect_payload = cmd_connect(self.config.host_major, self.config.host_minor);
                let reply = self.send_raw(&connect_payload, MessageType::Cmd).await?;
                match reply {
                    Response::Ack {
                        opcode,
                        payload: ack_payload,
                    } if opcode == CommandOpcode::Connect.as_byte()
                        && ack_payload.len() >= 2
                        && ack_payload[0] == self.config.host_major =>
                    {
                        self.connected.store(true, Ordering::SeqCst);
                        self.start_keepalive_locked().await;
                    }
                    _ => {
                        return Err(SessionError::Protocol(
                            "reconnect after BAD_SESSION failed — peer didn't accept CONNECT",
                        ));
                    }
                }
                self.send_raw(payload, MessageType::Cmd).await
            }
            _ => Ok(first),
        }
    }

    /// Like [`Session::send_command`], but retries on **transient**
    /// outcomes up to `max_attempts` total, with linear backoff.
    ///
    /// For the flash pipeline (FMEA #271 G11 + G12): before this, a
    /// single dropped frame, a transient adapter hiccup, or a slow
    /// erase that just missed the timeout would `?`-propagate and abort
    /// the whole flash mid-stream, leaving an erased-but-unwritten
    /// sector. Every command this is used for is **idempotent** —
    /// re-erasing a sector, re-reading a CRC, or re-writing the same
    /// bytes to already-erased flash is a no-op — so retrying is safe
    /// even when the failure happened *after* the command ran (a
    /// dropped ACK).
    ///
    /// "Transient" (see [`is_transient_send_outcome`]) covers a local
    /// `CommandTimeout` / transport error AND a NACK whose code is the
    /// bootloader's "I didn't receive the whole message" signal
    /// (`TRANSPORT_TIMEOUT` / `TRANSPORT_ERROR`) — the dropped-frame
    /// case on the fast adapters we don't 1 ms-pace (G12), far cheaper
    /// to retry than to pace every frame. A *definitive* NACK
    /// (CrcMismatch, BadSession, Unsupported, …) is returned at once,
    /// as are `RxClosed` / `Protocol`. The whole try→retry sequence
    /// holds `command_lock`. `max_attempts` is clamped to at least 1.
    pub async fn send_command_retrying(
        &self,
        payload: &[u8],
        max_attempts: u32,
    ) -> Result<Response, SessionError> {
        let _guard = self.inner.command_lock.lock().await;
        let attempts = max_attempts.max(1);
        for attempt in 1..=attempts {
            let outcome = self.send_raw(payload, MessageType::Cmd).await;
            if !is_transient_send_outcome(&outcome) || attempt == attempts {
                return outcome;
            }
            warn!(
                attempt,
                max_attempts = attempts,
                "flash command failed transiently (timeout / dropped-frame NACK) — \
                 backing off + retrying"
            );
            tokio::time::sleep(Duration::from_millis(
                u64::from(attempt) * COMMAND_RETRY_BACKOFF_MS,
            ))
            .await;
        }
        // Unreachable: the loop either returns the outcome on the final
        // attempt or returns a non-transient outcome early.
        unreachable!("send_command_retrying loop must return")
    }

    /// Broadcast a command (dst = `BROADCAST_NODE_ID`) and collect
    /// every reply that arrives within `collect_for`. Intended for
    /// `CMD_DISCOVER`; the usual reply is `Response::Discover`, but
    /// any reply the device emits in the window lands in the
    /// returned vec so callers can see stray ACKs / NACKs too.
    pub async fn broadcast(
        &self,
        payload: &[u8],
        message_type: MessageType,
        collect_for: Duration,
    ) -> Result<Vec<Response>, SessionError> {
        let _guard = self.inner.command_lock.lock().await;
        self.send_frames(payload, message_type, BROADCAST_NODE_ID)
            .await?;

        let deadline = Instant::now() + collect_for;
        let mut collected = Vec::new();
        let mut rx = self.inner.reply_rx.lock().await;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            match timeout(remaining, rx.recv()).await {
                Ok(Some(resp)) => collected.push(resp),
                Ok(None) => return Err(SessionError::RxClosed),
                Err(_elapsed) => break,
            }
        }
        Ok(collected)
    }

    /// Subscribe to the notification stream (`NOTIFY_HEARTBEAT`,
    /// `NOTIFY_DTC`, `NOTIFY_LOG`, `NOTIFY_LIVE_DATA`). Every active
    /// subscriber receives every notification; a slow subscriber
    /// receives `RecvError::Lagged` and can resync by calling
    /// `subscribe_notifications` again.
    pub fn subscribe_notifications(&self) -> broadcast::Receiver<Response> {
        self.inner.notification_tx.subscribe()
    }

    /// Send `CMD_DISCONNECT`, stop keepalive, tear down the RX task.
    /// Idempotent once called; after this the `Session` is unusable.
    ///
    /// Fire-and-forget on the wire: we don't wait for the device's ACK,
    /// because every single caller of `disconnect()` already discards
    /// whatever reply would come back. Waiting would cost a full
    /// `command_timeout` in the one case that matters — when the peer
    /// just jumped to the application via `CMD_JUMP` and the BL is
    /// no longer on the bus to ACK us. Firing CMD_DISCONNECT and
    /// tearing down locally without blocking matches the existing
    /// `let _ = …` pattern and makes the post-jump path finish in
    /// milliseconds instead of `command_timeout` seconds.
    pub async fn disconnect(self) -> Result<(), SessionError> {
        // Best-effort: acquire the command lock to play nicely with
        // concurrent send_command, but don't deadlock if we can't.
        let _guard = self.inner.command_lock.lock().await;
        if self.connected.load(Ordering::SeqCst) {
            let payload = cmd_disconnect();
            // Send the frame but don't wait for a reply. If the BL is
            // still alive it sees CMD_DISCONNECT and clears its session
            // latch; if the BL has jumped to the application the
            // frame's lost in the ether and that's fine. Any ACK that
            // does come back lands in the reply mpsc and is dropped
            // when we drop `self` below.
            let _ = self
                .send_frames(&payload, MessageType::Cmd, self.inner.target_node)
                .await;
            self.connected.store(false, Ordering::SeqCst);
        }
        self.stop_keepalive_locked().await;
        self.rx_shutdown.store(true, Ordering::SeqCst);
        if let Some(handle) = self.rx_handle.lock().await.take() {
            handle.abort();
            let _ = handle.await;
        }
        Ok(())
    }

    // ---- Internals ----

    /// Segment `payload` and transmit, then await a single reply
    /// within the configured command timeout. Caller must hold
    /// `command_lock`.
    async fn send_raw(
        &self,
        payload: &[u8],
        message_type: MessageType,
    ) -> Result<Response, SessionError> {
        self.send_raw_with_timeout(payload, message_type, self.config.command_timeout)
            .await
    }

    /// [`send_raw`](Self::send_raw) with an explicit reply timeout,
    /// instead of the session `command_timeout`. Lets the CONNECT
    /// handshake use a short per-attempt timeout while the erase-sized
    /// `command_timeout` still governs ordinary flash commands.
    async fn send_raw_with_timeout(
        &self,
        payload: &[u8],
        message_type: MessageType,
        reply_timeout: Duration,
    ) -> Result<Response, SessionError> {
        let errors_before = self.inner.backend.adapter_error_count();
        self.send_frames(payload, message_type, self.inner.target_node)
            .await?;
        let mut rx = self.inner.reply_rx.lock().await;
        match timeout(reply_timeout, rx.recv()).await {
            Ok(Some(response)) => Ok(response),
            Ok(None) => Err(SessionError::RxClosed),
            Err(_) => Err(SessionError::CommandTimeout {
                timeout: reply_timeout,
                adapter_errors_during_wait: self
                    .inner
                    .backend
                    .adapter_error_count()
                    .saturating_sub(errors_before),
            }),
        }
    }

    async fn send_frames(
        &self,
        payload: &[u8],
        message_type: MessageType,
        dst: u8,
    ) -> Result<(), SessionError> {
        // New wire format (fix/12): the ID no longer carries the
        // message type — it's prepended as the first byte of the
        // payload. Every frame (FF + CFs) shares the same
        // host→node ID; the PCI byte tells the receiver which frame
        // of the ISO-TP sequence it's looking at.
        let mut framed = Vec::with_capacity(1 + payload.len());
        framed.push(message_type.as_byte());
        framed.extend_from_slice(payload);

        let segmenter = IsoTpSegmenter::new(&framed).map_err(|e| {
            SessionError::Transport(TransportError::Other(format!(
                "session: payload rejected by segmenter: {e}"
            )))
        })?;
        let id = FrameId::from_host(dst)
            .expect("dst fits in 4 bits")
            .encode();

        for frame_bytes in segmenter {
            let frame = CanFrame {
                id,
                data: frame_bytes,
                len: frame_bytes.len() as u8,
            };
            self.inner.backend.send(frame).await?;
        }
        Ok(())
    }

    async fn start_keepalive(&self) {
        let mut guard = self.keepalive.lock().await;
        if guard.is_some() {
            return;
        }
        *guard = Some(self.spawn_keepalive_task());
    }

    async fn start_keepalive_locked(&self) {
        // Same body as start_keepalive but callable from contexts
        // that already hold command_lock (the reconnect path).
        let mut guard = self.keepalive.lock().await;
        if guard.is_some() {
            return;
        }
        *guard = Some(self.spawn_keepalive_task());
    }

    fn spawn_keepalive_task(&self) -> KeepaliveState {
        let inner = Arc::clone(&self.inner);
        let interval = self.config.keepalive_interval;
        let cmd_timeout = self.config.command_timeout;
        let (cancel_tx, mut cancel_rx) = oneshot::channel();
        let handle = tokio::spawn(async move {
            loop {
                tokio::select! {
                    biased;
                    _ = &mut cancel_rx => {
                        trace!("keepalive: cancel signalled");
                        return;
                    }
                    _ = tokio::time::sleep(interval) => {
                        if let Err(err) = keepalive_tick(&inner, cmd_timeout).await {
                            warn!(?err, "keepalive tick failed — stopping");
                            return;
                        }
                    }
                }
            }
        });
        KeepaliveState { cancel_tx, handle }
    }

    async fn stop_keepalive_locked(&self) {
        let taken = {
            let mut guard = self.keepalive.lock().await;
            guard.take()
        };
        if let Some(state) = taken {
            let _ = state.cancel_tx.send(());
            let _ = state.handle.await;
        }
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.rx_shutdown.store(true, Ordering::SeqCst);
        // Best-effort abort of the RX task. Can't .await in Drop; the
        // RX task's shutdown check + AbortHandle combine to kill it
        // promptly on the tokio executor.
        if let Ok(mut guard) = self.rx_handle.try_lock() {
            if let Some(h) = guard.take() {
                h.abort();
            }
        }
        if let Ok(mut guard) = self.keepalive.try_lock() {
            if let Some(state) = guard.take() {
                let _ = state.cancel_tx.send(());
                state.handle.abort();
            }
        }
    }
}

/// Single keepalive tick: issue `CMD_GET_HEALTH`, drop the response.
/// Refreshes the bootloader's 30 s session watchdog. Runs under its
/// own borrow of `command_lock` so it serialises with user-driven
/// commands.
async fn keepalive_tick(
    inner: &Arc<SessionInner>,
    command_timeout: Duration,
) -> Result<(), SessionError> {
    let _guard = inner.command_lock.lock().await;
    let errors_before = inner.backend.adapter_error_count();
    let payload = cmd_get_health();

    // Mirrors `send_frames` — prepend msg_type, single ID for FF+CFs.
    let mut framed = Vec::with_capacity(1 + payload.len());
    framed.push(MessageType::Cmd.as_byte());
    framed.extend_from_slice(&payload);

    let segmenter = IsoTpSegmenter::new(&framed).map_err(|e| {
        SessionError::Transport(TransportError::Other(format!(
            "keepalive: payload rejected by segmenter: {e}"
        )))
    })?;
    let id = FrameId::from_host(inner.target_node)
        .expect("target_node fits in 4 bits")
        .encode();
    for bytes in segmenter {
        let frame = CanFrame {
            id,
            data: bytes,
            len: bytes.len() as u8,
        };
        inner.backend.send(frame).await?;
    }

    let mut rx = inner.reply_rx.lock().await;
    match timeout(command_timeout, rx.recv()).await {
        Ok(Some(_resp)) => Ok(()),
        Ok(None) => Err(SessionError::RxClosed),
        Err(_) => Err(SessionError::CommandTimeout {
            timeout: command_timeout,
            adapter_errors_during_wait: inner
                .backend
                .adapter_error_count()
                .saturating_sub(errors_before),
        }),
    }
}

/// Should a command reply (ACK / NACK / DISCOVER) from source node
/// `src` be accepted by a session whose configured target is
/// `target`? (FMEA #271 G7.)
///
/// - point-to-point session (target = a specific node): only that
///   node's replies — a foreign node's same-opcode ACK must not be
///   allowed to satisfy our in-flight command.
/// - broadcast session (target = `BROADCAST_NODE_ID`, used by
///   `discover` + its per-node follow-up pings): accept every
///   responder.
///
/// Notifications are NOT gated by this — they're forwarded regardless
/// of source (consumers filter).
fn reply_source_accepted(src: u8, target: u8) -> bool {
    target == BROADCAST_NODE_ID || src == target
}

/// Should a [`Session::send_command_retrying`] outcome be retried?
///
/// Transient = the command provably didn't take effect and a resend
/// is safe for an idempotent command:
/// - a local `CommandTimeout` or transport error (FMEA #271 G11), or
/// - a NACK whose code is the bootloader's "I didn't receive the
///   whole message" signal — `TRANSPORT_TIMEOUT` / `TRANSPORT_ERROR`
///   — i.e. a dropped ISO-TP frame on a fast adapter we don't
///   1 ms-pace (FMEA #271 G12).
///
/// A *definitive* NACK (CrcMismatch, BadSession, Unsupported, …) and
/// the fatal `RxClosed` / `Protocol` errors are NOT transient — the
/// caller gets them immediately.
fn is_transient_send_outcome(outcome: &Result<Response, SessionError>) -> bool {
    matches!(
        outcome,
        Err(SessionError::CommandTimeout { .. } | SessionError::Transport(_))
            | Ok(Response::Nack {
                code: NackCode::TransportTimeout | NackCode::TransportError,
                ..
            })
    )
}

/// Background RX task. Owns the only read path into the backend,
/// decodes ISO-TP frames, routes completed `Response`s onto the
/// appropriate channel.
async fn rx_task(
    backend: Arc<dyn CanBackend>,
    reply_tx: mpsc::Sender<Response>,
    notification_tx: broadcast::Sender<Response>,
    node_id: u8,
    shutdown: Arc<AtomicBool>,
) {
    // FMEA #271 G7: one reassembler PER SOURCE NODE, not one shared.
    // The old single reassembler fed every NodeToHost frame, so a
    // foreign node's FF/CF on a multi-BL bus could splice into the
    // target's in-flight multi-frame reply (GET_FW_INFO / OB_READ /
    // CRC). Keyed by source node, each responder reassembles
    // independently. Bounded — at most one entry per 4-bit node id.
    let mut reassemblers: HashMap<u8, Reassembler> = HashMap::new();
    let mut tick_start = Instant::now();
    let tick_ms = move || tick_start.elapsed().as_millis() as u64;
    // Re-borrow because closures that capture mutable state can't
    // return it. Keep as a cheap inline expression.
    let _ = &mut tick_start;

    loop {
        if shutdown.load(Ordering::SeqCst) {
            trace!("session rx: shutdown flag set, exiting");
            return;
        }

        let frame = match backend.recv(Duration::from_millis(50)).await {
            Ok(f) => f,
            Err(TransportError::Timeout(_)) => continue,
            Err(TransportError::Disconnected) => {
                trace!("session rx: backend disconnected");
                return;
            }
            Err(err) => {
                warn!(?err, "session rx: backend error");
                return;
            }
        };

        // Drop frames with an invalid or not-for-us ID. Under the
        // Proposal-A layout, the host only cares about NodeToHost
        // frames (direction bit set). Host-originated frames on the
        // bus (our own TX echo) and malformed IDs get silently
        // dropped here.
        let id = match FrameId::decode(frame.id) {
            Ok(id) => id,
            Err(_) => continue,
        };
        if !matches!(
            id.direction,
            crate::protocol::ids::FrameDirection::NodeToHost
        ) {
            // Either our own TX echo (HostToNode) or a malformed
            // frame; the reassembler should never see these.
            continue;
        }

        // For NodeToHost, `id.node` is the responder (source).
        let src = id.node;
        let payload = frame.payload();

        let reasm = reassemblers.entry(src).or_default();
        match reasm.feed(payload, tick_ms()) {
            Ok(ReassembleOutcome::Ongoing) => continue,
            Ok(ReassembleOutcome::Complete(bytes)) => {
                // Every reassembled SF/FF message starts with the
                // msg_type byte (fix/12 wire format). Decode it,
                // then hand the remaining bytes to the response
                // parser. An empty reassembly is a bug; log and drop.
                if bytes.is_empty() {
                    warn!("session rx: empty reassembly — dropping");
                    continue;
                }
                let msg_type = match MessageType::from_byte(bytes[0]) {
                    Ok(mt) => mt,
                    Err(err) => {
                        warn!(
                            ?err,
                            byte = bytes[0],
                            "session rx: unknown msg_type — dropping"
                        );
                        continue;
                    }
                };
                let inner_bytes = &bytes[1..];
                match Response::parse(msg_type, inner_bytes) {
                    Ok(Response::Notify { .. }) => {
                        let response = Response::parse(msg_type, inner_bytes).unwrap();
                        // Notifications (log / live-data telemetry) are
                        // forwarded regardless of source — consumers
                        // filter — but lagged subscribers get
                        // RecvError::Lagged on their next recv; we
                        // don't treat overflow as an error here.
                        let _ = notification_tx.send(response);
                    }
                    Ok(other) => {
                        // FMEA #271 G7: a command reply (ACK / NACK /
                        // DISCOVER) only counts if it came from the node
                        // we're addressing. A foreign node's same-opcode
                        // ACK must NOT satisfy our in-flight command —
                        // that's how a wrong board's reply could be read
                        // as ours. EXCEPTION: a broadcast session
                        // (target = 0xF, used by `discover` + its
                        // per-node follow-up pings) legitimately collects
                        // replies from every node, so don't filter then.
                        if !reply_source_accepted(src, node_id) {
                            trace!(
                                src,
                                target = node_id,
                                "session rx: dropping command reply from non-target node"
                            );
                            continue;
                        }
                        if reply_tx.send(other).await.is_err() {
                            // No one is listening — command holder
                            // has dropped. Not fatal; keep running
                            // for notifications.
                        }
                    }
                    Err(err) => {
                        warn!(?err, "session rx: Response::parse failed; dropping frame");
                    }
                }
            }
            Err(err) => {
                warn!(?err, "session rx: reassembler error; resetting");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{StubDevice, VirtualBackend, VirtualBus};

    const STUB_NODE: u8 = 0x3;

    // FMEA #271 G7 — reply source filter.
    #[test]
    fn reply_source_filter_accepts_only_target_or_broadcast() {
        // Point-to-point: only the addressed node's replies count.
        assert!(reply_source_accepted(0x3, 0x3));
        assert!(!reply_source_accepted(0x5, 0x3));
        assert!(!reply_source_accepted(0x0, 0x3));
        // Broadcast session (discover): every responder is accepted.
        assert!(reply_source_accepted(0x1, BROADCAST_NODE_ID));
        assert!(reply_source_accepted(0x5, BROADCAST_NODE_ID));
        assert!(reply_source_accepted(0xE, BROADCAST_NODE_ID));
    }

    // FMEA #271 G11 + G12 — retry transient outcomes only.
    #[test]
    fn transient_send_outcome_classification() {
        let nack = |code| {
            Ok(Response::Nack {
                rejected_opcode: 0x10,
                code,
            })
        };
        // G11: local timeout + transport error.
        assert!(is_transient_send_outcome(&Err(
            SessionError::CommandTimeout {
                timeout: Duration::from_millis(1),
                adapter_errors_during_wait: 0,
            }
        )));
        assert!(is_transient_send_outcome(&Err(SessionError::Transport(
            TransportError::Disconnected
        ))));
        // G12: the bootloader's dropped-frame NACK codes.
        assert!(is_transient_send_outcome(&nack(NackCode::TransportTimeout)));
        assert!(is_transient_send_outcome(&nack(NackCode::TransportError)));
        // Definitive answers + fatal errors must NOT be retried.
        assert!(!is_transient_send_outcome(&nack(NackCode::CrcMismatch)));
        assert!(!is_transient_send_outcome(&nack(NackCode::BadSession)));
        assert!(!is_transient_send_outcome(&nack(NackCode::Unsupported)));
        assert!(!is_transient_send_outcome(&Err(SessionError::RxClosed)));
        assert!(!is_transient_send_outcome(&Ok(Response::Ack {
            opcode: 0x10,
            payload: vec![],
        })));
    }

    fn test_config() -> SessionConfig {
        SessionConfig {
            target_node: STUB_NODE,
            // Tight timings so tests run fast.
            keepalive_interval: Duration::from_millis(250),
            command_timeout: Duration::from_millis(200),
            host_major: PROTOCOL_VERSION_MAJOR,
            host_minor: PROTOCOL_VERSION_MINOR,
        }
    }

    async fn spawn_session_and_stub() -> (Session, oneshot::Sender<()>, JoinHandle<()>) {
        let bus = VirtualBus::new();
        let host = bus.host_backend();
        let device: Box<dyn CanBackend> = Box::new(bus.device_backend());
        drop(bus);

        let stub = StubDevice::new(device, STUB_NODE);
        let (cancel_tx, cancel_rx) = oneshot::channel();
        let handle = tokio::spawn(async move {
            let _ = stub.run(cancel_rx).await;
        });
        let session = Session::attach(Box::new(host), test_config());
        (session, cancel_tx, handle)
    }

    // ---- FMEA #271 G11: bounded retry on transient send failures ----

    /// Wraps a `VirtualBackend` and fails the next `fail_remaining`
    /// `send` calls with a transient transport error, then delegates
    /// normally. Lets us prove `send_command_retrying` rides out
    /// dropped frames / adapter hiccups instead of aborting.
    struct FlakyHost {
        inner: VirtualBackend,
        fail_remaining: Arc<std::sync::atomic::AtomicU32>,
    }

    #[async_trait::async_trait]
    impl CanBackend for FlakyHost {
        async fn send(&self, frame: CanFrame) -> crate::transport::Result<()> {
            // Decrement-if-positive: fail while we still owe failures.
            let mut cur = self.fail_remaining.load(Ordering::SeqCst);
            while cur > 0 {
                match self.fail_remaining.compare_exchange(
                    cur,
                    cur - 1,
                    Ordering::SeqCst,
                    Ordering::SeqCst,
                ) {
                    Ok(_) => {
                        return Err(TransportError::Other(
                            "injected transient send failure".into(),
                        ))
                    }
                    Err(actual) => cur = actual,
                }
            }
            self.inner.send(frame).await
        }
        async fn recv(&self, timeout: Duration) -> crate::transport::Result<CanFrame> {
            self.inner.recv(timeout).await
        }
        async fn set_bitrate(&self, nominal_bps: u32) -> crate::transport::Result<()> {
            self.inner.set_bitrate(nominal_bps).await
        }
        fn description(&self) -> String {
            self.inner.description()
        }
    }

    async fn spawn_session_and_stub_flaky() -> (
        Session,
        Arc<std::sync::atomic::AtomicU32>,
        oneshot::Sender<()>,
        JoinHandle<()>,
    ) {
        let bus = VirtualBus::new();
        let host = bus.host_backend();
        let device: Box<dyn CanBackend> = Box::new(bus.device_backend());
        drop(bus);

        let stub = StubDevice::new(device, STUB_NODE);
        let (cancel_tx, cancel_rx) = oneshot::channel();
        let handle = tokio::spawn(async move {
            let _ = stub.run(cancel_rx).await;
        });
        let fail_remaining = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let flaky = FlakyHost {
            inner: host,
            fail_remaining: Arc::clone(&fail_remaining),
        };
        // Long keepalive so a tick can't consume an injected failure
        // mid-test (keepalive only starts after connect anyway).
        let config = SessionConfig {
            keepalive_interval: Duration::from_secs(60),
            ..test_config()
        };
        let session = Session::attach(Box::new(flaky), config);
        (session, fail_remaining, cancel_tx, handle)
    }

    #[tokio::test]
    async fn send_command_retrying_rides_out_transient_send_failures() {
        let (session, fails, cancel, handle) = spawn_session_and_stub_flaky().await;
        session.connect().await.unwrap(); // clean connect (fails = 0)

        // Arm: the next 2 sends fail. With 3 attempts, the 3rd send
        // goes through and the stub NACKs the unknown opcode 0x20.
        fails.store(2, Ordering::SeqCst);
        let resp = session.send_command_retrying(&[0x20], 3).await.unwrap();
        assert!(
            matches!(
                resp,
                Response::Nack {
                    code: NackCode::Unsupported,
                    ..
                }
            ),
            "retry should have ridden out 2 drops and got the NACK, got {resp:?}"
        );
        assert_eq!(
            fails.load(Ordering::SeqCst),
            0,
            "both injected failures consumed"
        );

        // Sanity: plain send_command (no retry) fails on a single drop.
        fails.store(1, Ordering::SeqCst);
        assert!(
            session.send_command(&[0x20]).await.is_err(),
            "non-retrying send must surface the transient failure"
        );

        session.disconnect().await.unwrap();
        let _ = cancel.send(());
        let _ = handle.await;
    }

    #[tokio::test]
    async fn send_command_retrying_gives_up_after_max_attempts() {
        let (session, fails, cancel, handle) = spawn_session_and_stub_flaky().await;
        session.connect().await.unwrap();

        // More failures than attempts → exhaust + surface the error.
        fails.store(5, Ordering::SeqCst);
        let err = session.send_command_retrying(&[0x20], 3).await;
        assert!(
            matches!(err, Err(SessionError::Transport(_))),
            "should give up with the transport error, got {err:?}"
        );

        session.disconnect().await.unwrap();
        let _ = cancel.send(());
        let _ = handle.await;
    }

    #[tokio::test]
    async fn connect_succeeds_against_stub() {
        let (session, cancel, handle) = spawn_session_and_stub().await;
        let (major, minor) = session.connect().await.unwrap();
        assert_eq!(major, PROTOCOL_VERSION_MAJOR);
        assert_eq!(minor, PROTOCOL_VERSION_MINOR);
        assert!(session.is_connected());
        session.disconnect().await.unwrap();
        let _ = cancel.send(());
        let _ = handle.await;
    }

    #[tokio::test]
    async fn connect_with_bad_major_errors() {
        let (session, cancel, handle) = {
            let bus = VirtualBus::new();
            let host = bus.host_backend();
            let device: Box<dyn CanBackend> = Box::new(bus.device_backend());
            drop(bus);
            let stub = StubDevice::new(device, STUB_NODE);
            let (cancel_tx, cancel_rx) = oneshot::channel();
            let handle = tokio::spawn(async move {
                let _ = stub.run(cancel_rx).await;
            });
            // Override the host major to something the stub will NACK.
            let mut cfg = test_config();
            cfg.host_major = 99;
            let session = Session::attach(Box::new(host), cfg);
            (session, cancel_tx, handle)
        };

        let err = session.connect().await.unwrap_err();
        assert!(matches!(err, SessionError::ProtocolVersionMismatch { .. }));
        assert!(!session.is_connected());
        // Don't call disconnect() (session never connected); just drop.
        drop(session);
        let _ = cancel.send(());
        let _ = handle.await;
    }

    #[tokio::test]
    async fn send_command_to_unknown_opcode_returns_nack() {
        let (session, cancel, handle) = spawn_session_and_stub().await;
        session.connect().await.unwrap();
        // Opcode 0x20 isn't defined in `CommandOpcode` — the stub's
        // dispatch takes the `Err(_) → NACK(UNSUPPORTED)` branch.
        // (Every defined opcode has a stub handler now; testing the
        // "unknown opcode" fallthrough means reaching for a raw byte.)
        let payload = vec![0x20u8];
        let resp = session.send_command(&payload).await.unwrap();
        match resp {
            Response::Nack { code, .. } => assert_eq!(code, NackCode::Unsupported),
            other => panic!("expected Nack(Unsupported), got {other:?}"),
        }
        session.disconnect().await.unwrap();
        let _ = cancel.send(());
        let _ = handle.await;
    }

    #[tokio::test]
    async fn broadcast_discover_collects_single_reply() {
        let (session, cancel, handle) = spawn_session_and_stub().await;
        let payload = crate::protocol::commands::cmd_discover();
        let replies = session
            .broadcast(
                &payload,
                MessageType::DiscoverRequest,
                Duration::from_millis(150),
            )
            .await
            .unwrap();
        assert_eq!(replies.len(), 1, "one stub means one discover reply");
        match &replies[0] {
            Response::Discover { node_id, .. } => assert_eq!(*node_id, STUB_NODE),
            other => panic!("expected Discover, got {other:?}"),
        }
        let _ = cancel.send(());
        let _ = handle.await;
    }

    #[tokio::test]
    async fn command_times_out_when_no_peer() {
        // VirtualBus with both endpoints but no stub running.
        let bus = VirtualBus::new();
        let host = bus.host_backend();
        let _device = bus.device_backend(); // keep endpoint alive for channel
        drop(bus);

        let session = Session::attach(Box::new(host), test_config());
        let err = session.connect().await.unwrap_err();
        match err {
            SessionError::CommandTimeout {
                timeout,
                adapter_errors_during_wait,
            } => {
                assert_eq!(timeout, Duration::from_millis(200));
                assert_eq!(adapter_errors_during_wait, 0);
            }
            other => panic!("expected CommandTimeout, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn disconnect_is_idempotent_when_never_connected() {
        let (session, cancel, handle) = spawn_session_and_stub().await;
        session.disconnect().await.unwrap();
        let _ = cancel.send(());
        let _ = handle.await;
    }
}
