//! `diagnose` subcommand — DTC table, log ring, live-data snapshot,
//! health report, and remote reset.
//!
//! Runtime-inspection surface. Replaces the `debug` subcommand from
//! the pre-v1.0.0 draft: the bootloader doesn't expose raw memory
//! read/write, but it does stream the log ring (`NOTIFY_LOG`) and
//! the live-data snapshot (`NOTIFY_LIVE_DATA`) at host-configurable
//! rates, which covers the same observability need.
//!
//! ## Sub-subcommands
//!
//! | Sub | Session? | Wire |
//! |-----|:--------:|------|
//! | `health` | no | one-shot `CMD_GET_HEALTH` → 32-byte record |
//! | `read-dtc` | no | `CMD_DTC_READ` → `[count_le16, entries…]` |
//! | `clear-dtc` | **yes** | session-gated `CMD_DTC_CLEAR` + prompt unless `--yes` |
//! | `log` | **yes** | `CMD_LOG_STREAM_START(severity)` + subscribe to `NOTIFY_LOG` |
//! | `live-data` | **yes** | `CMD_LIVE_DATA_START(rate_hz)` + subscribe to `NOTIFY_LIVE_DATA` |
//! | `reset` | no | `CMD_RESET(mode)` — device reboots after ACK |
//!
//! Streaming subs (`log`, `live-data`) run until the user hits Ctrl-C,
//! at which point they send the matching `*_STOP` command and exit
//! cleanly.

use std::io::Write;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use clap::{Args, Subcommand, ValueEnum};
use serde::Serialize;
use tokio::sync::broadcast::error::RecvError;
use tracing::{debug, warn};

use super::GlobalFlags;
use crate::protocol::commands::{
    cmd_dtc_clear, cmd_dtc_read, cmd_get_health, cmd_live_data_start, cmd_live_data_stop,
    cmd_log_stream_start, cmd_log_stream_stop, cmd_reset,
};
use crate::protocol::opcodes::{NotifyOpcode, ResetMode as ProtoResetMode};
use crate::protocol::records::{DtcEntry, DtcSeverity, HealthRecord, LiveDataSnapshot, ResetCause};
use crate::protocol::Response;
use crate::session::{Session, SessionConfig};
use crate::transport::open_backend;

#[derive(Debug, Args)]
pub struct DiagnoseArgs {
    #[command(subcommand)]
    pub action: DiagnoseAction,
}

#[derive(Debug, Subcommand)]
pub enum DiagnoseAction {
    /// Read stored fault codes via CMD_DTC_READ
    ReadDtc,

    /// Clear stored fault codes via CMD_DTC_CLEAR (prompts unless --yes)
    ClearDtc {
        /// Skip the interactive confirmation prompt
        #[arg(long = "yes", default_value_t = false)]
        yes: bool,
    },

    /// Stream the bootloader log ring (CMD_LOG_STREAM_START + NOTIFY_LOG)
    Log {
        /// Minimum severity to emit (0=info, 1=warn, 2=error, 3=fatal)
        #[arg(long = "severity", default_value_t = 0)]
        severity: u8,
    },

    /// Stream the 32-byte live-data snapshot (CMD_LIVE_DATA_START + NOTIFY_LIVE_DATA)
    LiveData {
        /// Emission rate in Hz (1..=50)
        #[arg(long = "rate-hz", default_value_t = 10)]
        rate_hz: u8,
    },

    /// One-shot session health report (CMD_GET_HEALTH, 32-byte record)
    Health,

    /// Reset the device via CMD_RESET
    Reset {
        /// Reset mode
        #[arg(long = "mode", value_enum, default_value_t = ResetMode::Hard)]
        mode: ResetMode,
    },
}

/// CLI-side reset mode. Kept separate from [`ProtoResetMode`] so
/// clap can derive `ValueEnum` on a repr-less type and the CLI-level
/// help text carries our own language.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum ResetMode {
    /// NVIC_SystemReset (mode 0)
    Hard,
    /// Equivalent to hard on this family (mode 1, no distinction in HW)
    Soft,
    /// Reset into the bootloader's listen loop (mode 2, sets RTC BKP0R magic)
    Bootloader,
    /// Direct jump to the installed application (mode 3, no reset)
    App,
}

impl From<ResetMode> for ProtoResetMode {
    fn from(m: ResetMode) -> Self {
        match m {
            ResetMode::Hard => ProtoResetMode::Hard,
            ResetMode::Soft => ProtoResetMode::Soft,
            ResetMode::Bootloader => ProtoResetMode::Bootloader,
            ResetMode::App => ProtoResetMode::App,
        }
    }
}

pub async fn run(args: DiagnoseArgs, global: &GlobalFlags) -> Result<()> {
    match args.action {
        DiagnoseAction::Health => run_health(global).await,
        DiagnoseAction::ReadDtc => run_read_dtc(global).await,
        DiagnoseAction::ClearDtc { yes } => run_clear_dtc(yes, global).await,
        DiagnoseAction::Log { severity } => run_log(severity, global).await,
        DiagnoseAction::LiveData { rate_hz } => run_live_data(rate_hz, global).await,
        DiagnoseAction::Reset { mode } => run_reset(mode, global).await,
    }
}

// ---- Session helpers ----

/// Build a session from global flags. Used by every diagnose arm —
/// keeps config tweaks in one place.
fn open_session(global: &GlobalFlags) -> Result<Session> {
    let backend = open_backend(global.interface, global.channel.as_deref(), global.bitrate)
        .context("opening CAN backend for diagnose")?;
    // target_node mirrors --node-id when specified; otherwise the
    // session stays at 0x0 (unused until send_command_to picks a
    // real dst). For diagnose commands we route everything at
    // --node-id so the single-node case just works.
    let target_node = global.node_id.unwrap_or(0x3);
    let config = SessionConfig {
        target_node,
        keepalive_interval: Duration::from_millis(5_000),
        command_timeout: Duration::from_millis(u64::from(global.timeout_ms)),
        ..SessionConfig::default()
    };
    Ok(Session::attach(backend, config))
}

/// Session-less one-shot: open session, send, tear down. Doesn't
/// call `connect()`.
async fn oneshot(global: &GlobalFlags, payload: Vec<u8>) -> Result<Response> {
    let session = open_session(global)?;
    let resp = session
        .send_command(&payload)
        .await
        .context("sending command")?;
    let _ = session.disconnect().await;
    Ok(resp)
}

// ---- HEALTH ----

async fn run_health(global: &GlobalFlags) -> Result<()> {
    debug!("diagnose health: starting");
    let resp = oneshot(global, cmd_get_health()).await?;
    match resp {
        Response::Ack { payload, .. } => {
            // `Response::Ack.payload` already has the echoed opcode
            // stripped by the parser, so the record starts at byte 0.
            if payload.len() < HealthRecord::SIZE {
                bail!(
                    "GET_HEALTH ACK too short: got {} bytes, need at least {}",
                    payload.len(),
                    HealthRecord::SIZE
                );
            }
            let record = HealthRecord::parse(&payload)
                .context("parsing HealthRecord from GET_HEALTH ACK")?;
            render_health(&record, global.json)
        }
        Response::Nack {
            rejected_opcode,
            code,
        } => bail!("device NACK'd GET_HEALTH (opcode 0x{rejected_opcode:02X}): {code}"),
        other => bail!("unexpected reply to GET_HEALTH: {}", other.kind_str()),
    }
}

/// Serialisable snapshot of the 32-byte `HealthRecord` with its
/// enum / bool fields already decoded. This is what flows out of
/// `--json` and what the human renderer pulls from too, so the two
/// paths stay honest with each other.
#[derive(Debug, Serialize)]
struct HealthJson {
    uptime_seconds: u32,
    reset_cause: String,
    reset_cause_raw: u32,
    flash_write_count: u32,
    dtc_count: u32,
    last_dtc_code: u32,
    fdcan_recovery_count: u32,
    max_flash_op_ms: u32,
    session_active: bool,
    valid_app_present: bool,
    wrp_protected: bool,
    raw_flags: u32,
}

impl HealthJson {
    fn from_record(r: &HealthRecord) -> Self {
        Self {
            uptime_seconds: r.uptime_seconds,
            reset_cause: r
                .reset_cause()
                .map(|c: ResetCause| c.as_str().to_string())
                .unwrap_or_else(|| format!("UNKNOWN (0x{:02X})", r.reset_cause)),
            reset_cause_raw: r.reset_cause,
            flash_write_count: r.flash_write_count,
            dtc_count: r.dtc_count,
            last_dtc_code: r.last_dtc_code,
            fdcan_recovery_count: r.fdcan_recovery_count,
            max_flash_op_ms: r.max_flash_op_ms,
            session_active: r.session_active(),
            valid_app_present: r.valid_app_present(),
            wrp_protected: r.wrp_protected(),
            raw_flags: r.flags,
        }
    }
}

fn render_health(record: &HealthRecord, json: bool) -> Result<()> {
    let snapshot = HealthJson::from_record(record);
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    if json {
        serde_json::to_writer_pretty(&mut out, &snapshot)
            .context("serialising health record as JSON")?;
        writeln!(out).ok();
        return Ok(());
    }

    let uptime_hms = format_uptime_seconds(snapshot.uptime_seconds);
    writeln!(out, "Device health")?;
    writeln!(out, "─────────────")?;
    writeln!(
        out,
        "  Uptime         : {}s ({uptime_hms})",
        snapshot.uptime_seconds
    )?;
    writeln!(out, "  Reset cause    : {}", snapshot.reset_cause)?;
    writeln!(
        out,
        "  Session active : {}",
        if snapshot.session_active { "yes" } else { "no" }
    )?;
    writeln!(
        out,
        "  Valid app      : {}",
        if snapshot.valid_app_present {
            "yes"
        } else {
            "no"
        }
    )?;
    writeln!(
        out,
        "  WRP protected  : {}",
        if snapshot.wrp_protected { "yes" } else { "no" }
    )?;
    writeln!(out, "  Flash writes   : {}", snapshot.flash_write_count)?;
    writeln!(out, "  DTC count      : {}", snapshot.dtc_count)?;
    writeln!(out, "  Last DTC code  : 0x{:04X}", snapshot.last_dtc_code)?;
    writeln!(out, "  FDCAN recovery : {}", snapshot.fdcan_recovery_count)?;
    writeln!(out, "  Max flash op   : {} ms", snapshot.max_flash_op_ms)?;
    writeln!(out, "  Raw flags      : 0x{:08X}", snapshot.raw_flags)?;
    Ok(())
}

fn format_uptime_seconds(s: u32) -> String {
    let h = s / 3600;
    let m = (s % 3600) / 60;
    let sec = s % 60;
    if h > 0 {
        format!("{h}h{m:02}m{sec:02}s")
    } else if m > 0 {
        format!("{m}m{sec:02}s")
    } else {
        format!("{sec}s")
    }
}

// ---- READ-DTC ----

async fn run_read_dtc(global: &GlobalFlags) -> Result<()> {
    debug!("diagnose read-dtc: starting");
    let resp = oneshot(global, cmd_dtc_read()).await?;
    match resp {
        Response::Ack { payload, .. } => {
            // `Response::Ack.payload` has the opcode stripped; so
            // the record starts at byte 0: `[count_le16, entry_0, …]`.
            if payload.len() < 2 {
                bail!(
                    "DTC_READ ACK too short: got {} bytes, need at least 2 for count",
                    payload.len()
                );
            }
            let count = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            let mut entries = Vec::with_capacity(count);
            for i in 0..count {
                let off = 2 + i * DtcEntry::SIZE;
                if off + DtcEntry::SIZE > payload.len() {
                    bail!(
                        "DTC_READ ACK truncated at entry {i}: payload has {} bytes",
                        payload.len()
                    );
                }
                entries.push(DtcEntry::parse(&payload[off..off + DtcEntry::SIZE])?);
            }
            render_dtc_table(&entries, global.json)
        }
        Response::Nack {
            rejected_opcode,
            code,
        } => bail!("device NACK'd DTC_READ (opcode 0x{rejected_opcode:02X}): {code}"),
        other => bail!("unexpected reply to DTC_READ: {}", other.kind_str()),
    }
}

#[derive(Debug, Serialize)]
struct DtcJson {
    code: u16,
    severity: String,
    severity_raw: u8,
    occurrence_count: u8,
    first_seen_uptime_seconds: u32,
    last_seen_uptime_seconds: u32,
    context_data: u32,
}

fn render_dtc_table(entries: &[DtcEntry], json: bool) -> Result<()> {
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    if json {
        let rows: Vec<DtcJson> = entries
            .iter()
            .map(|e| DtcJson {
                code: e.code,
                severity: e.severity().as_str().to_string(),
                severity_raw: e.severity,
                occurrence_count: e.occurrence_count,
                first_seen_uptime_seconds: e.first_seen_uptime_seconds,
                last_seen_uptime_seconds: e.last_seen_uptime_seconds,
                context_data: e.context_data,
            })
            .collect();
        serde_json::to_writer_pretty(&mut out, &rows).context("serialising DTC table as JSON")?;
        writeln!(out).ok();
        return Ok(());
    }

    if entries.is_empty() {
        writeln!(out, "No DTCs logged.")?;
        return Ok(());
    }

    writeln!(
        out,
        "{:<8}  {:<9}  {:<5}  {:>12}  {:>12}  {:<10}",
        "Code", "Severity", "Count", "First seen", "Last seen", "Context"
    )?;
    writeln!(
        out,
        "{}  {}  {}  {}  {}  {}",
        "─".repeat(8),
        "─".repeat(9),
        "─".repeat(5),
        "─".repeat(12),
        "─".repeat(12),
        "─".repeat(10),
    )?;
    for e in entries {
        let sev_label = match e.severity() {
            DtcSeverity::Info => "INFO".to_string(),
            DtcSeverity::Warn => "WARN".to_string(),
            DtcSeverity::Error => "ERROR".to_string(),
            DtcSeverity::Fatal => "FATAL".to_string(),
            DtcSeverity::Unknown(x) => format!("?({x:#04X})"),
        };
        writeln!(
            out,
            "0x{:04X}    {:<9}  {:>5}  {:>10}s  {:>10}s  0x{:08X}",
            e.code,
            sev_label,
            e.occurrence_count,
            e.first_seen_uptime_seconds,
            e.last_seen_uptime_seconds,
            e.context_data,
        )?;
    }
    Ok(())
}

// ---- CLEAR-DTC ----

async fn run_clear_dtc(yes: bool, global: &GlobalFlags) -> Result<()> {
    if !yes && !confirm_prompt("This will clear every DTC entry on the device. Continue?") {
        bail!("cancelled");
    }

    let session = open_session(global)?;
    session
        .connect()
        .await
        .context("CONNECT before DTC_CLEAR")?;

    let resp = session
        .send_command(&cmd_dtc_clear())
        .await
        .context("sending DTC_CLEAR");

    // Always attempt disconnect — even on error path — so we don't
    // strand a session on the device.
    let disconnect_result = session.disconnect().await;

    let resp = resp?;
    disconnect_result.ok();

    match resp {
        Response::Ack { .. } => {
            println!("DTC table cleared.");
            Ok(())
        }
        Response::Nack {
            rejected_opcode,
            code,
        } => bail!("device NACK'd DTC_CLEAR (opcode 0x{rejected_opcode:02X}): {code}"),
        other => bail!("unexpected reply to DTC_CLEAR: {}", other.kind_str()),
    }
}

fn confirm_prompt(question: &str) -> bool {
    eprint!("{question} [y/N]: ");
    std::io::stderr().flush().ok();
    let mut input = String::new();
    if std::io::stdin().read_line(&mut input).is_err() {
        return false;
    }
    matches!(input.trim().to_lowercase().as_str(), "y" | "yes")
}

// ---- RESET ----

async fn run_reset(mode: ResetMode, global: &GlobalFlags) -> Result<()> {
    let proto_mode: ProtoResetMode = mode.into();
    let resp = oneshot(global, cmd_reset(proto_mode)).await?;
    match resp {
        Response::Ack { .. } => {
            // The device ACKs and then reboots. We don't expect a
            // follow-up response.
            let label = match mode {
                ResetMode::Hard => "hard (NVIC_SystemReset)",
                ResetMode::Soft => "soft (same as hard on this family)",
                ResetMode::Bootloader => "into bootloader listen mode",
                ResetMode::App => "jump to installed app",
            };
            if global.json {
                println!(r#"{{"status":"ok","mode":"{:?}"}}"#, mode);
            } else {
                println!("Reset issued — mode: {label}");
            }
            Ok(())
        }
        Response::Nack {
            rejected_opcode,
            code,
        } => bail!("device NACK'd RESET (opcode 0x{rejected_opcode:02X}): {code}"),
        other => bail!("unexpected reply to RESET: {}", other.kind_str()),
    }
}

// ---- LOG STREAM ----

async fn run_log(severity: u8, global: &GlobalFlags) -> Result<()> {
    let session = open_session(global)?;
    session
        .connect()
        .await
        .context("CONNECT before LOG_STREAM_START")?;

    let mut subscriber = session.subscribe_notifications();

    match session
        .send_command(&cmd_log_stream_start(severity))
        .await?
    {
        Response::Ack { .. } => {}
        Response::Nack {
            rejected_opcode,
            code,
        } => {
            let _ = session.disconnect().await;
            bail!("device NACK'd LOG_STREAM_START (opcode 0x{rejected_opcode:02X}): {code}");
        }
        other => {
            let _ = session.disconnect().await;
            bail!("unexpected reply to LOG_STREAM_START: {}", other.kind_str());
        }
    }

    eprintln!("Streaming log (min severity {severity}) — press Ctrl-C to stop.");

    let shutdown = async {
        let _ = tokio::signal::ctrl_c().await;
    };
    tokio::pin!(shutdown);

    loop {
        tokio::select! {
            _ = &mut shutdown => {
                eprintln!("\nstopping log stream…");
                break;
            }
            res = subscriber.recv() => match res {
                Ok(Response::Notify { opcode, payload }) if opcode == NotifyOpcode::Log.as_byte() => {
                    render_log_entry(&payload);
                }
                Ok(_) => {} // other notify kinds — ignored here
                Err(RecvError::Lagged(n)) => {
                    warn!(dropped = n, "log stream: subscriber lagged, some entries skipped");
                }
                Err(RecvError::Closed) => break,
            }
        }
    }

    let _ = session.send_command(&cmd_log_stream_stop()).await;
    let _ = session.disconnect().await;
    Ok(())
}

/// Decode and print a single `NOTIFY_LOG` payload. Each notification
/// carries one or more log-ring entries drained from the bootloader;
/// the on-wire format is documented in `bl_log.h` (severity byte +
/// flags byte + 4-byte timestamp + ASCII chunk up to the remaining
/// frame length).
///
/// Best-effort renderer for feat/11: prints severity label, timestamp,
/// and a hex+UTF-8 view of the chunk. Later feat branches can
/// re-assemble multi-frame log lines if needed — for bootloader v1.0.0
/// log entries fit in a single `NOTIFY_LOG` so there's nothing to
/// reassemble.
fn render_log_entry(payload: &[u8]) {
    // payload[0] is the notify opcode stripped by the caller, so our
    // `payload` starts at the log entry's own bytes:
    //   [severity, flags, ts_le32, message…]
    if payload.len() < 6 {
        println!("(log entry truncated: {} bytes)", payload.len());
        return;
    }
    let severity = payload[0];
    let flags = payload[1];
    let ts = u32::from_le_bytes([payload[2], payload[3], payload[4], payload[5]]);
    let msg_bytes = &payload[6..];

    let sev_label = match severity {
        0 => "INFO",
        1 => "WARN",
        2 => "ERROR",
        3 => "FATAL",
        _ => "?",
    };

    let msg = String::from_utf8_lossy(msg_bytes.split(|b| *b == 0).next().unwrap_or(msg_bytes));
    println!("[{ts:>10}ms] {sev_label:<5} [flags=0x{flags:02X}] {msg}");
}

// ---- LIVE-DATA STREAM ----

async fn run_live_data(rate_hz: u8, global: &GlobalFlags) -> Result<()> {
    if !(1..=50).contains(&rate_hz) {
        bail!("--rate-hz must be in 1..=50 (got {rate_hz})");
    }

    let session = open_session(global)?;
    session
        .connect()
        .await
        .context("CONNECT before LIVE_DATA_START")?;

    let mut subscriber = session.subscribe_notifications();

    match session.send_command(&cmd_live_data_start(rate_hz)).await? {
        Response::Ack { .. } => {}
        Response::Nack {
            rejected_opcode,
            code,
        } => {
            let _ = session.disconnect().await;
            bail!("device NACK'd LIVE_DATA_START (opcode 0x{rejected_opcode:02X}): {code}");
        }
        other => {
            let _ = session.disconnect().await;
            bail!("unexpected reply to LIVE_DATA_START: {}", other.kind_str());
        }
    }

    eprintln!("Streaming live-data @ {rate_hz} Hz — press Ctrl-C to stop.");

    let shutdown = async {
        let _ = tokio::signal::ctrl_c().await;
    };
    tokio::pin!(shutdown);

    loop {
        tokio::select! {
            _ = &mut shutdown => {
                eprintln!("\nstopping live-data stream…");
                break;
            }
            res = subscriber.recv() => match res {
                Ok(Response::Notify { opcode, payload }) if opcode == NotifyOpcode::LiveData.as_byte() => {
                    render_live_data_snapshot(&payload, global.json);
                }
                Ok(_) => {}
                Err(RecvError::Lagged(n)) => {
                    warn!(dropped = n, "live-data: subscriber lagged, some snapshots skipped");
                }
                Err(RecvError::Closed) => break,
            }
        }
    }

    let _ = session.send_command(&cmd_live_data_stop()).await;
    let _ = session.disconnect().await;
    Ok(())
}

fn render_live_data_snapshot(payload: &[u8], json: bool) {
    match LiveDataSnapshot::parse(payload) {
        Ok(snap) => {
            if json {
                let row = live_snapshot_to_json(&snap);
                if let Ok(s) = serde_json::to_string(&row) {
                    println!("{s}");
                }
            } else {
                println!(
                    "[{:>10}ms] rx={:>5} tx={:>5} nacks={:>3} dtc={} last_op=0x{:02X} last_flash=0x{:08X} isotp_prog={} sess_age={}ms flags=0x{:02X}",
                    snap.uptime_ms,
                    snap.frames_rx,
                    snap.frames_tx,
                    snap.nacks_sent,
                    snap.dtc_count,
                    snap.last_opcode,
                    snap.last_flash_addr,
                    snap.isotp_rx_progress,
                    snap.session_age_ms,
                    snap.flags,
                );
            }
        }
        Err(err) => warn!(?err, "live-data: failed to parse snapshot"),
    }
}

#[derive(Debug, Serialize)]
struct LiveDataJson {
    uptime_ms: u32,
    frames_rx: u16,
    frames_tx: u16,
    nacks_sent: u16,
    dtc_count: u16,
    last_dtc_code: u16,
    flags: u8,
    last_opcode: u8,
    last_flash_addr: u32,
    isotp_rx_progress: u32,
    session_age_ms: u32,
    session_active: bool,
    valid_app_present: bool,
    log_streaming: bool,
    livedata_streaming: bool,
    wrp_protected: bool,
}

fn live_snapshot_to_json(s: &LiveDataSnapshot) -> LiveDataJson {
    LiveDataJson {
        uptime_ms: s.uptime_ms,
        frames_rx: s.frames_rx,
        frames_tx: s.frames_tx,
        nacks_sent: s.nacks_sent,
        dtc_count: s.dtc_count,
        last_dtc_code: s.last_dtc_code,
        flags: s.flags,
        last_opcode: s.last_opcode,
        last_flash_addr: s.last_flash_addr,
        isotp_rx_progress: s.isotp_rx_progress,
        session_age_ms: s.session_age_ms,
        session_active: s.session_active(),
        valid_app_present: s.valid_app_present(),
        log_streaming: s.log_streaming(),
        livedata_streaming: s.livedata_streaming(),
        wrp_protected: s.wrp_protected(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_uptime_seconds_handles_ranges() {
        assert_eq!(format_uptime_seconds(0), "0s");
        assert_eq!(format_uptime_seconds(42), "42s");
        assert_eq!(format_uptime_seconds(90), "1m30s");
        assert_eq!(format_uptime_seconds(3700), "1h01m40s");
    }

    #[test]
    fn reset_mode_conversion_matches_protocol_bytes() {
        assert_eq!(ProtoResetMode::from(ResetMode::Hard).as_byte(), 0);
        assert_eq!(ProtoResetMode::from(ResetMode::Soft).as_byte(), 1);
        assert_eq!(ProtoResetMode::from(ResetMode::Bootloader).as_byte(), 2);
        assert_eq!(ProtoResetMode::from(ResetMode::App).as_byte(), 3);
    }

    fn sample_record() -> HealthRecord {
        HealthRecord {
            uptime_seconds: 3600 + 42,
            reset_cause: ResetCause::PowerOn as u32,
            flags: 0b00010011, // SESSION + VALID_APP + WRP
            flash_write_count: 17,
            dtc_count: 2,
            last_dtc_code: 0x0010,
            fdcan_recovery_count: 3,
            max_flash_op_ms: 1200,
        }
    }

    #[test]
    fn health_json_decodes_flag_bits() {
        let s = HealthJson::from_record(&sample_record());
        assert!(s.session_active);
        assert!(s.valid_app_present);
        assert!(s.wrp_protected);
        assert_eq!(s.reset_cause, "POWER_ON");
    }

    #[test]
    fn health_json_serialises_with_all_fields() {
        let s = HealthJson::from_record(&sample_record());
        let json = serde_json::to_string(&s).unwrap();
        for key in [
            "\"uptime_seconds\":",
            "\"reset_cause\":",
            "\"session_active\":",
            "\"valid_app_present\":",
            "\"wrp_protected\":",
            "\"fdcan_recovery_count\":",
            "\"max_flash_op_ms\":",
            "\"raw_flags\":",
        ] {
            assert!(json.contains(key), "missing {key}: {json}");
        }
    }
}
