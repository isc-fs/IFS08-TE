//! `cf pit-diag` — terminal-side pit-diag observer (AMS / ECU / uDV).
//!
//! Four subcommands:
//!
//! ```text
//! cf pit-diag enable                 # arm the stream
//! cf pit-diag disable                # disarm the stream
//! cf pit-diag stream [--json]        # arm + stream + disarm-on-exit
//! cf pit-diag listen [--json]        # passive: decode ungated frames, never arm
//! ```
//!
//! Wraps the `pit_diag` library module (handshake + decoders) with a
//! tokio runtime + a Ctrl-C / duration-bound loop. Useful for:
//!
//!  - Bench validation without firing up MingoCAN. Pipe `stream
//!    --json` through `jq` to grep specific fields, or watch the
//!    human-readable form roll past on a serial terminal.
//!  - CI smoke checks. The virtual-bus stub can play canned frames
//!    and this binary decodes them, exiting non-zero on schema drift.
//!  - A reference decoder MingoCAN could shell out to in a pinch
//!    (the Tauri command currently runs in-process; if the workspace
//!    ever splits further, this CLI is the canonical implementation).
//!
//! The `--profile` flag selects the board: `ams` (arm `0x7F0`, decode
//! `0x680–0x6CA`), `ecu` (arm `0x7E0`, decode `0x700–0x707`), or `udv`
//! (arm `0x7DE`, decode `0x7A0–0x7A6`).

use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use clap::{Args, Subcommand};

use crate::cli::{GlobalFlags, InterfaceType};
use crate::pit_diag::ecu;
use crate::pit_diag::udv;
use crate::pit_diag::{
    build_arm_frame, decode_frame, FaultReason, PitDiagFrame, AMS_ACK_ID,
    AMS_EXPECTED_FRAMES_PER_SCAN,
};
use crate::transport::{open_backend, CanBackend, TransportError};

/// Which board's pit-diag stream a run targets. Each uses different arm
/// IDs and frame layouts, so the arm handshake + decoder are chosen per
/// profile: AMS (`0x7F0`, ACK `0x7F1`), ECU (`0x7E0`, ACK `0x7E1`), uDV
/// (`0x7DE`, sticky — no ACK, no disarm).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Profile {
    Ams,
    Ecu,
    Udv,
}

fn parse_profile(profile: &str) -> Result<Profile> {
    match profile {
        "ams" => Ok(Profile::Ams),
        "ecu" => Ok(Profile::Ecu),
        "udv" => Ok(Profile::Udv),
        other => Err(anyhow!(
            "unknown pit-diag profile {other:?}: supported profiles are 'ams', 'ecu', and 'udv'"
        )),
    }
}

/// How long to wait for the AMS to ACK an arm / disarm command. The
/// firmware ACKs within ~10ms in the happy case; 2s leaves room for
/// a noisy bus + slow USB adapter without making `cf pit-diag enable`
/// feel unresponsive.
const ACK_TIMEOUT: Duration = Duration::from_millis(2_000);

/// Per-recv timeout inside the stream loop. Short enough that the
/// Ctrl-C / duration check happens within one cycle.
const POLL_TIMEOUT: Duration = Duration::from_millis(50);

// ---- Argument parsing -------------------------------------------

#[derive(Debug, Args)]
pub struct PitDiagArgs {
    #[command(subcommand)]
    pub command: PitDiagCommand,
}

#[derive(Debug, Subcommand)]
pub enum PitDiagCommand {
    /// Arm the pit-diag stream — sends `0x7F0#DEADBEEF` and waits
    /// for the `0x7F1` ACK. Idempotent: re-arming an already-armed
    /// stream is a no-op on the firmware side.
    Enable(ProfileArgs),

    /// Disarm the pit-diag stream — sends `0x7F0#00000000` and waits
    /// for the `0x7F1` ACK. The firmware also clears the flag on
    /// reboot, so this is belt-and-braces against a tool crash.
    Disable(ProfileArgs),

    /// Arm, decode the stream to stdout, then disarm on exit.
    ///
    /// Default output is one human-readable line per decoded record.
    /// `--json` switches to NDJSON (one JSON object per line) for
    /// pipe-into-jq workflows. Exit reasons:
    ///
    ///   - `--duration` seconds elapsed → exit 0
    ///   - Ctrl-C (SIGINT)              → exit 0, with disarm
    ///   - Bus error                    → exit non-zero
    ///   - Schema drift (frames/scan
    ///     diverges from expected 58)   → exit non-zero
    Stream(StreamArgs),

    /// Passively decode the stream to stdout WITHOUT arming.
    ///
    /// Unlike `stream`, this never sends the `0x7E0`/`0x7F0` arm frame —
    /// it only receives. It exists for the frames a board broadcasts
    /// ungated: the ECU app emits `0x704` health at 1 Hz with no arm
    /// required (task-liveness bits, reset_cause, last_fault, uptime,
    /// heap). Because it's send-silent it's safe to run against a live
    /// car, and it answers "is this board's app alive?" the instant the
    /// board powers up. `--json` emits the same NDJSON as `stream`
    /// (`ecuHealth`/…), so the same consumer parses either. Exits on
    /// `--duration-ms`, Ctrl-C, or bus error.
    Listen(ListenArgs),
}

#[derive(Debug, Args)]
pub struct ProfileArgs {
    /// Which board's stream to address: `ams` (0x7F0), `ecu` (0x7E0),
    /// or `udv` (0x7DE).
    #[arg(long, default_value = "ams")]
    pub profile: String,
}

#[derive(Debug, Args)]
pub struct StreamArgs {
    /// Which board's stream to address: `ams` (0x7F0), `ecu` (0x7E0),
    /// or `udv` (0x7DE).
    #[arg(long, default_value = "ams")]
    pub profile: String,

    /// Stop after this many seconds. Omit to stream until Ctrl-C.
    #[arg(long)]
    pub duration: Option<u64>,

    /// Fail the run if any 1-second window's frame count drifts from
    /// the expected per-profile total (58 AMS, 7 ECU, 4 uDV) by more
    /// than ±2. Off by default because operators inspecting a
    /// known-broken bus want to *see* the wrong count, not have the
    /// tool bail. Enable in CI / scripted bench checks.
    #[arg(long, default_value_t = false)]
    pub strict_scan: bool,
}

#[derive(Debug, Args)]
pub struct ListenArgs {
    /// Which board's frames to decode: `ecu` (0x700–0x707), `ams`
    /// (incl. the ungated 0x6CA health), or `all`/`both` to decode
    /// whichever is on the bus. Defaults to `all` — a passive listen
    /// wants to hear any board's ungated health (ECU 0x704 / AMS 0x6CA).
    #[arg(long, default_value = "all")]
    pub profile: String,

    /// Stop after this many milliseconds. Omit to listen until Ctrl-C
    /// (the mode a live health indicator uses). A one-shot presence
    /// probe wants ~1500 ms — long enough to catch a 1 Hz health frame.
    #[arg(long)]
    pub duration_ms: Option<u64>,
}

// ---- Dispatch ---------------------------------------------------

pub async fn run(args: PitDiagArgs, global: &GlobalFlags) -> Result<()> {
    match args.command {
        PitDiagCommand::Enable(a) => run_enable(a, global).await,
        PitDiagCommand::Disable(a) => run_disable(a, global).await,
        PitDiagCommand::Stream(a) => run_stream(a, global).await,
        PitDiagCommand::Listen(a) => run_listen(a, global).await,
    }
}

// ---- enable / disable -------------------------------------------

async fn run_enable(args: ProfileArgs, global: &GlobalFlags) -> Result<()> {
    let profile = parse_profile(&args.profile)?;
    let backend = open(global)?;
    arm(&*backend, profile, true)
        .await
        .with_context(|| format!("arming {} pit-diag stream", args.profile))?;
    print_ok(global.json, &args.profile, true);
    Ok(())
}

async fn run_disable(args: ProfileArgs, global: &GlobalFlags) -> Result<()> {
    let profile = parse_profile(&args.profile)?;
    let backend = open(global)?;
    arm(&*backend, profile, false)
        .await
        .with_context(|| format!("disarming {} pit-diag stream", args.profile))?;
    print_ok(global.json, &args.profile, false);
    Ok(())
}

fn print_ok(json: bool, profile: &str, enabled: bool) {
    if json {
        println!(r#"{{"profile":"{profile}","enabled":{enabled}}}"#);
    } else {
        let verb = if enabled { "armed" } else { "disarmed" };
        println!("✓ {profile} pit-diag {verb}");
    }
}

// ---- stream -----------------------------------------------------

async fn run_stream(args: StreamArgs, global: &GlobalFlags) -> Result<()> {
    let profile = parse_profile(&args.profile)?;
    let backend = open(global)?;

    arm(&*backend, profile, true)
        .await
        .with_context(|| format!("arming {} pit-diag stream", args.profile))?;
    if !global.json {
        eprintln!("✓ {} pit-diag armed — streaming…", args.profile);
    }

    // Race the stream loop against Ctrl-C + the optional duration.
    // Whatever completes first triggers a graceful shutdown.
    let started = Instant::now();
    let duration = args.duration.map(Duration::from_secs);

    let loop_result = tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            if !global.json {
                eprintln!("\n— Ctrl-C — disarming");
            }
            Ok(())
        }
        result = stream_loop(&*backend, profile, &args, started, duration, global.json) => result,
    };

    // Disarm best-effort, even after a bus error in the loop. A
    // failed disarm just means the board keeps streaming until the
    // operator power-cycles it — annoying but not dangerous.
    let disarm_result = arm(&*backend, profile, false).await;
    if !global.json {
        match &disarm_result {
            Ok(()) => eprintln!("✓ {} pit-diag disarmed", args.profile),
            Err(err) => eprintln!("⚠ disarm failed: {err}"),
        }
    }

    loop_result?;
    Ok(())
}

// ---- listen (passive) -------------------------------------------

async fn run_listen(args: ListenArgs, global: &GlobalFlags) -> Result<()> {
    // `None` = decode both boards (the `all`/`both` default); `Some(p)` pins
    // one. Passive listen is send-silent so decoding everything is free —
    // ECU IDs (0x7xx) and AMS IDs (0x6xx/0x4xx) never overlap.
    let decode = match args.profile.to_ascii_lowercase().as_str() {
        "all" | "both" => None,
        _ => Some(parse_profile(&args.profile)?),
    };
    let backend = open(global)?;

    // No arm — this path is deliberately send-silent so it can run
    // against a live car without perturbing it.
    if !global.json {
        eprintln!("• {} pit-diag passive listen (no arm)…", args.profile);
    }

    let started = Instant::now();
    let duration = args.duration_ms.map(Duration::from_millis);

    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            if !global.json {
                eprintln!("\n— Ctrl-C —");
            }
            Ok(())
        }
        result = listen_loop(&*backend, decode, started, duration, global.json) => result,
    }
}

/// Inner passive-listen loop — decodes received frames until duration /
/// bus error, without ever transmitting. Mirrors `stream_loop`'s decode
/// dispatch minus the arm-dependent scan-rate accounting (a passive
/// listener sees only whatever the board broadcasts ungated, so a
/// per-scan frame count is meaningless). Ctrl-C is handled one level up.
async fn listen_loop(
    backend: &dyn CanBackend,
    decode: Option<Profile>,
    started: Instant,
    duration: Option<Duration>,
    json: bool,
) -> Result<()> {
    // `None` = the `all` default (try every decoder); `Some(p)` pins one.
    let (want_ams, want_ecu, want_udv) = match decode {
        None => (true, true, true),
        Some(Profile::Ams) => (true, false, false),
        Some(Profile::Ecu) => (false, true, false),
        Some(Profile::Udv) => (false, false, true),
    };
    loop {
        if let Some(d) = duration {
            if started.elapsed() >= d {
                return Ok(());
            }
        }

        match backend.recv(POLL_TIMEOUT).await {
            Ok(frame) => {
                let ts_ms = started.elapsed().as_millis() as u64;
                // Try each enabled decoder; IDs don't overlap, so at most one
                // matches. Both enabled = the `all` default.
                if want_ecu {
                    if let Some(record) = ecu::decode_frame(&frame) {
                        if json {
                            print_ecu_json(ts_ms, &record);
                        } else {
                            print_ecu_human(ts_ms, &record);
                        }
                        continue;
                    }
                }
                if want_ams {
                    if let Some(record) = decode_frame(&frame) {
                        if json {
                            print_record_json(ts_ms, &record);
                        } else {
                            print_record_human(ts_ms, &record);
                        }
                        continue;
                    }
                }
                if want_udv {
                    if let Some(record) = udv::decode_frame(&frame) {
                        if json {
                            print_udv_json(ts_ms, &record);
                        } else {
                            print_udv_human(ts_ms, &record);
                        }
                    }
                }
                // Non-pit-diag frames are silently dropped.
            }
            Err(TransportError::Timeout(_)) => {
                // Expected between broadcasts — keep polling.
            }
            Err(err) => {
                return Err(anyhow!("bus error: {err}"));
            }
        }
    }
}

/// Inner stream loop — reads frames until duration / strict-scan
/// trip / bus error. Ctrl-C is handled one level up via `select!`.
async fn stream_loop(
    backend: &dyn CanBackend,
    profile: Profile,
    args: &StreamArgs,
    started: Instant,
    duration: Option<Duration>,
    json: bool,
) -> Result<()> {
    // Scan-rate tracking — mirrors the Studio's schema-drift safety
    // net. Each second we snapshot the running tally and reset.
    let expected_per_scan = match profile {
        Profile::Ams => AMS_EXPECTED_FRAMES_PER_SCAN,
        Profile::Ecu => ecu::ECU_EXPECTED_FRAMES_PER_SCAN,
        Profile::Udv => udv::UDV_EXPECTED_FRAMES_PER_SCAN,
    };
    let mut frames_this_scan: usize = 0;
    let mut last_scan_at = Instant::now();

    loop {
        if let Some(d) = duration {
            if started.elapsed() >= d {
                return Ok(());
            }
        }

        // 1-second scan-rate roll. Done before the recv so a slow
        // bus doesn't delay the roll indefinitely.
        if last_scan_at.elapsed() >= Duration::from_secs(1) {
            if args.strict_scan
                && frames_this_scan > 0
                && frames_this_scan.abs_diff(expected_per_scan) > 2
            {
                return Err(anyhow!(
                    "schema drift: last scan had {frames_this_scan} frames, expected {expected_per_scan} (±2). \
                     The firmware's pit-diag wire shape may have changed since this tool \
                     was built — check src/pit_diag against the DBCinator source."
                ));
            }
            frames_this_scan = 0;
            last_scan_at = Instant::now();
        }

        match backend.recv(POLL_TIMEOUT).await {
            Ok(frame) => {
                let ts_ms = started.elapsed().as_millis() as u64;
                // Decode + print with the profile's decoder. Don't count
                // the ACK frame — it's not part of the cyclic scan.
                match profile {
                    Profile::Ams => {
                        if let Some(record) = decode_frame(&frame) {
                            if !matches!(record, PitDiagFrame::Ack { .. }) {
                                frames_this_scan += 1;
                            }
                            if json {
                                print_record_json(ts_ms, &record);
                            } else {
                                print_record_human(ts_ms, &record);
                            }
                        }
                    }
                    Profile::Ecu => {
                        if let Some(record) = ecu::decode_frame(&frame) {
                            if !matches!(record, ecu::EcuPitDiagFrame::Ack { .. }) {
                                frames_this_scan += 1;
                            }
                            if json {
                                print_ecu_json(ts_ms, &record);
                            } else {
                                print_ecu_human(ts_ms, &record);
                            }
                        }
                    }
                    Profile::Udv => {
                        if let Some(record) = udv::decode_frame(&frame) {
                            // fwinfo (~1 Hz) + calib (calibration-only) aren't
                            // part of the cyclic scan.
                            if !matches!(
                                record,
                                udv::UdvPitDiagFrame::FwInfo(_) | udv::UdvPitDiagFrame::Calib(_)
                            ) {
                                frames_this_scan += 1;
                            }
                            if json {
                                print_udv_json(ts_ms, &record);
                            } else {
                                print_udv_human(ts_ms, &record);
                            }
                        }
                    }
                }
                // Non-pit-diag frames are silently dropped — the
                // bus monitor is the place to see other traffic.
            }
            Err(TransportError::Timeout(_)) => {
                // Expected during the gaps between scans — keep
                // polling.
            }
            Err(err) => {
                return Err(anyhow!("bus error: {err}"));
            }
        }
    }
}

// ---- Output formatting ------------------------------------------

fn print_record_human(ts_ms: u64, record: &PitDiagFrame) {
    let prefix = format!("[+{:>7.3}s]", (ts_ms as f64) / 1000.0);
    match record {
        PitDiagFrame::Ack { enabled } => {
            eprintln!("{prefix} ack enabled={enabled}");
        }
        PitDiagFrame::CellVoltage(c) => {
            let cells = c
                .voltages_mv
                .iter()
                .map(|mv| format!("{mv:>4}"))
                .collect::<Vec<_>>()
                .join(" ");
            println!(
                "{prefix} cell  frame={:>2} cells[{:>2}..{:>2}] = {cells} mV",
                c.frame_idx,
                c.first_cell,
                c.first_cell + 4,
            );
        }
        PitDiagFrame::NtcTemp(t) => {
            let temps = t
                .temps_c
                .iter()
                .map(|c| format!("{c:>4}"))
                .collect::<Vec<_>>()
                .join(" ");
            println!(
                "{prefix} ntc   frame={:>2} ntcs[{:>3}..{:>3}] = {temps} °C",
                t.frame_idx,
                t.first_ntc,
                t.first_ntc + 8,
            );
        }
        PitDiagFrame::FsmStatus(f) => {
            // Only print the fault columns when something's latched —
            // keeps the happy-path line short.
            let fault = if matches!(f.fault_reason, FaultReason::None) {
                String::new()
            } else {
                format!(" fault={:?} detail={}", f.fault_reason, f.fault_detail)
            };
            println!(
                "{prefix} fsm   state={:?} mode={:?} tsms={} dash={} ams_ok={} pec={}{fault}",
                f.state,
                f.mode_locked,
                f.tsms as u8,
                f.dash_chg as u8,
                f.ams_ok as u8,
                f.pec_error_total,
            );
        }
        PitDiagFrame::PollTiming(p) => {
            println!(
                "{prefix} poll  v_last={:>4}ms v_worst={:>4}ms tsweep_fail=0x{:08X}",
                p.last_v_poll_ms, p.worst_v_poll_ms, p.t_sweep_fail_mask,
            );
        }
        PitDiagFrame::BalanceMaskA(b) => {
            println!("{prefix} bal-a cells[ 0..63] dcc=0x{:016X}", b.dcc_lo);
        }
        PitDiagFrame::BalanceMaskB(b) => {
            println!(
                "{prefix} bal-b cells[64..94] dcc=0x{:08X} cycles total={} active={}",
                b.dcc_hi, b.cycles_total, b.cycles_active,
            );
        }
        PitDiagFrame::BootDiag(b) => {
            println!(
                "{prefix} boot  jump={:?} init_progress={}/7 fdcan_start=0x{:06X}",
                b.jump_reason, b.app_init_progress, b.fdcan1_start_result,
            );
        }
        PitDiagFrame::PostMortem(p) => {
            if p.is_clean() {
                println!("{prefix} crash (clean — no fault on previous boot)");
            } else {
                println!(
                    "{prefix} crash stack_overflow={} watermark={} task_addr=0x{:08X} malloc_fails={}",
                    p.stack_overflow_seen as u8,
                    p.watermark_low_byte,
                    p.task_addr_lo,
                    p.malloc_failed_count,
                );
            }
        }
        PitDiagFrame::FwId(f) => {
            println!(
                "{prefix} fw    v{}.{}.{} git={:02X}{:02X}{:02X}{:02X} bl_node=0x{:02X}",
                f.version_major,
                f.version_minor,
                f.version_patch,
                f.git_hash[0],
                f.git_hash[1],
                f.git_hash[2],
                f.git_hash[3],
                f.bl_node_id,
            );
        }
        PitDiagFrame::PerIcPec(p) => {
            let counts = p
                .counts
                .iter()
                .take(p.valid as usize)
                .map(|c| format!("{c:>3}"))
                .collect::<Vec<_>>()
                .join(" ");
            println!(
                "{prefix} pec   ICs[{:>2}..{:>2}] = {counts}",
                p.first_ic,
                p.first_ic + p.valid,
            );
        }
        PitDiagFrame::RelayStatus(r) => {
            println!(
                "{prefix} relay AIR-={} AIR+={} precharge={} AMS_OK={}",
                r.air_negative as u8, r.air_positive as u8, r.precharge as u8, r.ams_ok as u8,
            );
        }
        PitDiagFrame::AcuCurrents(c) => {
            println!(
                "{prefix} curr  accu={:.1}A dcdc={:.1}A",
                f64::from(c.accu_da) * 0.1,
                f64::from(c.dcdc_da) * 0.1,
            );
        }
        PitDiagFrame::Pack(p) => {
            println!(
                "{prefix} pack  {:.3}V {:.1}A",
                f64::from(p.pack_voltage_mv) / 1000.0,
                f64::from(p.filtered_ma) / 1000.0,
            );
        }
        PitDiagFrame::Health(h) => {
            println!(
                "{prefix} health heap={}/{} tasks[main={} rx={} tx={} hk={}] reset={:?} \
                 uptime={}s last_fault=0x{:02X}",
                h.free_heap,
                h.min_free_heap,
                h.task_main as u8,
                h.task_can_rx as u8,
                h.task_can_tx as u8,
                h.task_housekeeping as u8,
                ecu::EcuResetCause::from_byte(h.reset_cause),
                h.uptime_s,
                h.last_fault,
            );
        }
    }
}

fn print_record_json(ts_ms: u64, record: &PitDiagFrame) {
    // Hand-written JSON to keep the line format stable + avoid a
    // serde_json transitive dep dragging into the CLI's already-lean
    // dependency tree. Field names match the Studio's
    // PitDiagEvent serde shape so any consumer scripting against
    // either side can share parsers.
    match record {
        PitDiagFrame::Ack { enabled } => {
            println!(r#"{{"tsMs":{ts_ms},"kind":"ack","enabled":{enabled}}}"#);
        }
        PitDiagFrame::CellVoltage(c) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"cellVoltage","frameIdx":{},"firstCell":{},"voltagesMv":[{}]}}"#,
                c.frame_idx,
                c.first_cell,
                c.voltages_mv
                    .iter()
                    .map(u16::to_string)
                    .collect::<Vec<_>>()
                    .join(","),
            );
        }
        PitDiagFrame::NtcTemp(t) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"ntcTemp","frameIdx":{},"firstNtc":{},"tempsC":[{}]}}"#,
                t.frame_idx,
                t.first_ntc,
                t.temps_c
                    .iter()
                    .map(i8::to_string)
                    .collect::<Vec<_>>()
                    .join(","),
            );
        }
        PitDiagFrame::FsmStatus(f) => {
            // FSM state / mode_locked / fault_reason are typed enums
            // on this side; mirror the Studio's stringified shape so
            // consumers can switch on the value without a translation
            // table.
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"fsmStatus","state":"{:?}","modeLocked":"{:?}","tsms":{},"dashChg":{},"amsOk":{},"pecErrorTotal":{},"faultReason":"{:?}","faultDetail":{}}}"#,
                f.state,
                f.mode_locked,
                f.tsms,
                f.dash_chg,
                f.ams_ok,
                f.pec_error_total,
                f.fault_reason,
                f.fault_detail,
            );
        }
        PitDiagFrame::PollTiming(p) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"pollTiming","lastVPollMs":{},"worstVPollMs":{},"tSweepFailMask":{}}}"#,
                p.last_v_poll_ms, p.worst_v_poll_ms, p.t_sweep_fail_mask,
            );
        }
        PitDiagFrame::BalanceMaskA(b) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"balanceMaskA","dccLo":{}}}"#,
                b.dcc_lo,
            );
        }
        PitDiagFrame::BalanceMaskB(b) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"balanceMaskB","dccHi":{},"cyclesTotal":{},"cyclesActive":{}}}"#,
                b.dcc_hi, b.cycles_total, b.cycles_active,
            );
        }
        PitDiagFrame::BootDiag(b) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"bootDiag","jumpReason":"{:?}","appInitProgress":{},"fdcan1StartResult":{}}}"#,
                b.jump_reason, b.app_init_progress, b.fdcan1_start_result,
            );
        }
        PitDiagFrame::PostMortem(p) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"postMortem","stackOverflowSeen":{},"watermarkLowByte":{},"taskAddrLo":{},"mallocFailedCount":{}}}"#,
                p.stack_overflow_seen, p.watermark_low_byte, p.task_addr_lo, p.malloc_failed_count,
            );
        }
        PitDiagFrame::FwId(f) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"fwId","versionMajor":{},"versionMinor":{},"versionPatch":{},"gitHash":[{},{},{},{}],"blNodeId":{}}}"#,
                f.version_major,
                f.version_minor,
                f.version_patch,
                f.git_hash[0],
                f.git_hash[1],
                f.git_hash[2],
                f.git_hash[3],
                f.bl_node_id,
            );
        }
        PitDiagFrame::PerIcPec(p) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"perIcPec","firstIc":{},"valid":{},"counts":[{}]}}"#,
                p.first_ic,
                p.valid,
                p.counts
                    .iter()
                    .take(p.valid as usize)
                    .map(u8::to_string)
                    .collect::<Vec<_>>()
                    .join(","),
            );
        }
        PitDiagFrame::RelayStatus(r) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"relayStatus","airNegative":{},"airPositive":{},"precharge":{},"amsOk":{}}}"#,
                r.air_negative, r.air_positive, r.precharge, r.ams_ok,
            );
        }
        PitDiagFrame::AcuCurrents(c) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"acuCurrents","accuDa":{},"dcdcDa":{}}}"#,
                c.accu_da, c.dcdc_da,
            );
        }
        PitDiagFrame::Pack(p) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"pack","packVoltageMv":{},"filteredMa":{}}}"#,
                p.pack_voltage_mv, p.filtered_ma,
            );
        }
        // Field names match `ecuHealth` (taskControl/…/taskDiag) so the same
        // consumer parser handles both boards' health uniformly.
        PitDiagFrame::Health(h) => {
            println!(
                r#"{{"tsMs":{ts_ms},"kind":"amsHealth","freeHeap":{},"minFreeHeap":{},"taskControl":{},"taskCanRx":{},"taskCanTx":{},"taskDiag":{},"resetCause":"{:?}","uptimeS":{},"lastFault":{}}}"#,
                h.free_heap,
                h.min_free_heap,
                h.task_main,
                h.task_can_rx,
                h.task_can_tx,
                h.task_housekeeping,
                ecu::EcuResetCause::from_byte(h.reset_cause),
                h.uptime_s,
                h.last_fault,
            );
        }
    }
}

// ---- ECU output formatting --------------------------------------

fn print_ecu_human(ts_ms: u64, record: &ecu::EcuPitDiagFrame) {
    use ecu::EcuPitDiagFrame as F;
    let prefix = format!("[+{:>7.3}s]", (ts_ms as f64) / 1000.0);
    match record {
        F::Ack { enabled } => eprintln!("{prefix} ack enabled={enabled}"),
        F::Status(s) => println!(
            "{prefix} status fsm={:?} inv={:?} torque={}% vcell_min={}mV tcmd={} \
             [ev23={} t11={} rtds={} preok={} start={} dv={}]",
            s.fsm_state,
            s.inv_state,
            s.torque_pct,
            s.v_cell_min_mv,
            s.torque_cmd,
            s.ev_2_3 as u8,
            s.t11_8_9 as u8,
            s.rtds_active as u8,
            s.ok_precharge as u8,
            s.start_button as u8,
            s.dv_mode as u8,
        ),
        F::Pedals(p) => println!(
            "{prefix} pedals apps1={}({}%) apps2={}({}%) brake_raw={}",
            p.apps1_raw, p.apps1_pct, p.apps2_raw, p.apps2_pct, p.brake_raw,
        ),
        F::Brake(b) => println!(
            "{prefix} brake {:.1}bar ({}%)",
            f64::from(b.brake_pressure_dbar) * 0.1,
            b.brake_pct,
        ),
        F::Inverter(i) => println!(
            "{prefix} inv   vdc={}V rpm={} err={} ({}) {}",
            i.dc_bus_voltage,
            i.inv_rpm,
            i.inv_error,
            ecu::dem_fault_name(i.inv_error),
            if i.dem_present { "active" } else { "latched" },
        ),
        F::InverterTemps(t) => println!(
            "{prefix} temps board={}°C pwrstg={}°C motor1={}°C motor2={}°C",
            t.board_degc, t.pwrstg_degc, t.motor1_degc, t.motor2_degc,
        ),
        F::FwInfo(f) => println!(
            "{prefix} fw    v{}.{}.{} git={:02X}{:02X}{:02X}{:02X}",
            f.fw_major,
            f.fw_minor,
            f.fw_patch,
            f.git_hash[0],
            f.git_hash[1],
            f.git_hash[2],
            f.git_hash[3],
        ),
        F::Health(h) => println!(
            "{prefix} health heap={}/{} tasks[ctrl={} rx={} tx={} diag={}] reset={:?} \
             uptime={}s last_fault=0x{:02X}",
            h.free_heap,
            h.min_free_heap,
            h.task_control as u8,
            h.task_can_rx as u8,
            h.task_can_tx as u8,
            h.task_diag as u8,
            h.reset_cause,
            h.uptime_s,
            h.last_fault,
        ),
        F::Dv(d) => println!(
            "{prefix} dv    mode_torque={}% rpm={} [r2d_req={} cmd_fresh={} ts={} brk_lim={} r2d_ok={}]",
            d.dv_torque_pct,
            d.motor_rpm_mech,
            d.dv_r2d_req as u8,
            d.dv_cmd_fresh as u8,
            d.ts_active as u8,
            d.brake_over_limit as u8,
            d.r2d_confirm as u8,
        ),
    }
}

/// NDJSON for the ECU profile. `kind` names match the Studio's
/// `PitDiagEvent` shape (`ecuStatus`/`ecuPedals`/…) so the same consumer
/// parser — e.g. the VS Code extension's live-monitor — works against
/// either transport. Hand-written to keep the CLI's dep tree lean.
fn print_ecu_json(ts_ms: u64, record: &ecu::EcuPitDiagFrame) {
    use ecu::EcuPitDiagFrame as F;
    match record {
        F::Ack { enabled } => {
            println!(r#"{{"tsMs":{ts_ms},"kind":"ack","enabled":{enabled}}}"#);
        }
        F::Status(s) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuStatus","fsmState":"{:?}","invState":"{:?}","ev23":{},"t11_8_9":{},"rtdsActive":{},"okPrecharge":{},"startButton":{},"dvMode":{},"torquePct":{},"vCellMinMv":{},"torqueCmd":{}}}"#,
            s.fsm_state,
            s.inv_state,
            s.ev_2_3,
            s.t11_8_9,
            s.rtds_active,
            s.ok_precharge,
            s.start_button,
            s.dv_mode,
            s.torque_pct,
            s.v_cell_min_mv,
            s.torque_cmd,
        ),
        F::Pedals(p) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuPedals","apps1Raw":{},"apps2Raw":{},"brakeRaw":{},"apps1Pct":{},"apps2Pct":{}}}"#,
            p.apps1_raw, p.apps2_raw, p.brake_raw, p.apps1_pct, p.apps2_pct,
        ),
        F::Brake(b) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuBrake","brakePressureDbar":{},"brakePct":{}}}"#,
            b.brake_pressure_dbar, b.brake_pct,
        ),
        F::Inverter(i) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuInverter","dcBusVoltage":{},"invRpm":{},"invError":{},"invErrorName":"{}","demPresent":{}}}"#,
            i.dc_bus_voltage,
            i.inv_rpm,
            i.inv_error,
            ecu::dem_fault_name(i.inv_error),
            i.dem_present,
        ),
        F::InverterTemps(t) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuInverterTemps","boardDegc":{},"pwrstgDegc":{},"motor1Degc":{},"motor2Degc":{}}}"#,
            t.board_degc, t.pwrstg_degc, t.motor1_degc, t.motor2_degc,
        ),
        F::FwInfo(f) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuFwInfo","fwMajor":{},"fwMinor":{},"fwPatch":{},"gitHash":[{},{},{},{}]}}"#,
            f.fw_major,
            f.fw_minor,
            f.fw_patch,
            f.git_hash[0],
            f.git_hash[1],
            f.git_hash[2],
            f.git_hash[3],
        ),
        F::Health(h) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuHealth","freeHeap":{},"minFreeHeap":{},"taskControl":{},"taskCanRx":{},"taskCanTx":{},"taskDiag":{},"resetCause":"{:?}","uptimeS":{},"lastFault":{}}}"#,
            h.free_heap,
            h.min_free_heap,
            h.task_control,
            h.task_can_rx,
            h.task_can_tx,
            h.task_diag,
            h.reset_cause,
            h.uptime_s,
            h.last_fault,
        ),
        F::Dv(d) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"ecuDv","dvR2dReq":{},"dvCmdFresh":{},"tsActive":{},"brakeOverLimit":{},"r2dConfirm":{},"dvTorquePct":{},"motorRpmMech":{}}}"#,
            d.dv_r2d_req,
            d.dv_cmd_fresh,
            d.ts_active,
            d.brake_over_limit,
            d.r2d_confirm,
            d.dv_torque_pct,
            d.motor_rpm_mech,
        ),
    }
}

// ---- uDV output formatting --------------------------------------

fn print_udv_human(ts_ms: u64, record: &udv::UdvPitDiagFrame) {
    use udv::UdvPitDiagFrame as F;
    let prefix = format!("[+{:>7.3}s]", (ts_ms as f64) / 1000.0);
    match record {
        F::Status(s) => println!(
            "{prefix} status as={:?} sig=0x{:04X} mission={} ebs={:?} assi={:?} armed={}",
            s.as_state, s.signals, s.mission_id, s.ebs_init, s.assi, s.diag_armed as u8,
        ),
        F::Res(r) => println!(
            "{prefix} res    status={:?} bits=0x{:02X} radio={} age={}ms steer={:?} raw191=0x{:02X}",
            r.res_status, r.bits, r.radio_quality, r.res_age_ms, r.steer_motor, r.raw_0x191,
        ),
        F::Pipe(p) => println!(
            "{prefix} pipe   dv=0x{:02X} accel={}% steer={} dv_age={}ms ctrl_age={}ms setup=0x{:02X}",
            p.dv_status, p.accel_cmd_pct, p.steer_cmd, p.dv_age_ms, p.ctrl_age_ms, p.setup_bits,
        ),
        F::Health(h) => println!(
            "{prefix} health heap={}/{}w tasks=0x{:02X} flags=0x{:02X} stalled={} uptime={}s",
            h.free_heap_words,
            h.min_free_heap_words,
            h.task_mask,
            h.flags,
            h.stalled_task,
            h.uptime_s,
        ),
        F::FwInfo(f) => println!(
            "{prefix} fw     git={:08X} stub=0x{:02X} heap={}KB uptime={}s",
            f.git_hash, f.stub_mask, f.heap_size_kb, f.uptime_s,
        ),
        F::CanHealth(c) => println!(
            "{prefix} canhlt flags=0x{:02X} lec={} tec={} rec={} res_rx={} nmt={} ack_err={}",
            c.flags, c.last_error_code, c.tx_err_count, c.rx_err_count, c.res_rx_count,
            c.nmt_count, c.ack_error as u8,
        ),
        F::Calib(c) => println!(
            "{prefix} calib  phase={} ({}) error={} ({}) center={:.1}° half={:.1}° limit={:.1}°",
            c.phase,
            udv::calib_phase_name(c.phase),
            c.error,
            udv::calib_error_name(c.error),
            f64::from(c.center_ddeg) * 0.1,
            f64::from(c.half_range_ddeg) * 0.1,
            f64::from(c.limit_ddeg) * 0.1,
        ),
        F::Steer(s) => println!(
            "{prefix} steer  lws={:.1}° actual={:.1}° target={:.1}° status=0x{:02X} motor={} ({})",
            f64::from(s.lws_raw_ddeg) * 0.1,
            f64::from(s.steer_actual_ddeg) * 0.1,
            f64::from(s.steer_target_ddeg) * 0.1,
            s.lws_status,
            s.motor_state,
            udv::steer_motor_state_name(s.motor_state),
        ),
        F::CalibRelay(r) => println!(
            "{prefix} crelay trig_rx={} relayed={} last_cmd=0x{:02X} armed={}",
            r.trigger_rx_count, r.relay_count, r.last_cmd, r.armed as u8,
        ),
        F::EbsPress(e) => println!(
            "{prefix} ebs    tank1={:.1}bar{} tank2={:.1}bar{} init={:?} stub=0x{:02X}",
            f64::from(e.tank1_dbar) * 0.1,
            if e.tank1_ok { " ok" } else { "" },
            f64::from(e.tank2_dbar) * 0.1,
            if e.tank2_ok { " ok" } else { "" },
            e.ebs_init,
            e.stub_mask,
        ),
    }
}

/// NDJSON for the uDV profile. `kind` names follow the `ecu*` convention
/// (`udvStatus`/`udvRes`/…). Bit masks are emitted raw (the consumer
/// decodes individual bits), enums as their debug names.
fn print_udv_json(ts_ms: u64, record: &udv::UdvPitDiagFrame) {
    use udv::UdvPitDiagFrame as F;
    match record {
        F::Status(s) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvStatus","asState":"{:?}","signals":{},"missionId":{},"ebsInit":"{:?}","stubMask":{},"assi":"{:?}","diagArmed":{}}}"#,
            s.as_state, s.signals, s.mission_id, s.ebs_init, s.stub_mask, s.assi, s.diag_armed,
        ),
        F::Res(r) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvRes","raw0x191":{},"resStatus":"{:?}","bits":{},"radioQuality":{},"resAgeMs":{},"steerMotor":"{:?}","lwsStatus":{}}}"#,
            r.raw_0x191,
            r.res_status,
            r.bits,
            r.radio_quality,
            r.res_age_ms,
            r.steer_motor,
            r.lws_status,
        ),
        F::Pipe(p) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvPipe","dvStatus":{},"dvAgeMs":{},"accelCmdPct":{},"steerCmd":{},"ctrlAgeMs":{},"setupBits":{}}}"#,
            p.dv_status, p.dv_age_ms, p.accel_cmd_pct, p.steer_cmd, p.ctrl_age_ms, p.setup_bits,
        ),
        F::Health(h) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvHealth","freeHeapWords":{},"minFreeHeapWords":{},"taskMask":{},"flags":{},"stalledTask":{},"uptimeS":{}}}"#,
            h.free_heap_words,
            h.min_free_heap_words,
            h.task_mask,
            h.flags,
            h.stalled_task,
            h.uptime_s,
        ),
        F::FwInfo(f) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvFwInfo","gitHash":{},"stubMask":{},"heapSizeKb":{},"uptimeS":{}}}"#,
            f.git_hash, f.stub_mask, f.heap_size_kb, f.uptime_s,
        ),
        F::CanHealth(c) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvCanHealth","flags":{},"lastErrorCode":{},"txErrCount":{},"rxErrCount":{},"resRxCount":{},"nmtCount":{},"ackError":{}}}"#,
            c.flags,
            c.last_error_code,
            c.tx_err_count,
            c.rx_err_count,
            c.res_rx_count,
            c.nmt_count,
            c.ack_error,
        ),
        F::Calib(c) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvCalib","phase":{},"error":{},"centerDdeg":{},"halfRangeDdeg":{},"limitDdeg":{}}}"#,
            c.phase, c.error, c.center_ddeg, c.half_range_ddeg, c.limit_ddeg,
        ),
        F::Steer(s) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvSteer","lwsRawDdeg":{},"steerActualDdeg":{},"steerTargetDdeg":{},"lwsStatus":{},"motorState":{}}}"#,
            s.lws_raw_ddeg, s.steer_actual_ddeg, s.steer_target_ddeg, s.lws_status, s.motor_state,
        ),
        F::CalibRelay(r) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvCalibRelay","triggerRxCount":{},"relayCount":{},"lastCmd":{},"armed":{}}}"#,
            r.trigger_rx_count, r.relay_count, r.last_cmd, r.armed,
        ),
        F::EbsPress(e) => println!(
            r#"{{"tsMs":{ts_ms},"kind":"udvEbsPress","tank1Dbar":{},"tank2Dbar":{},"ebsInit":"{:?}","stubMask":{},"tank1Ok":{},"tank2Ok":{}}}"#,
            e.tank1_dbar, e.tank2_dbar, e.ebs_init, e.stub_mask, e.tank1_ok, e.tank2_ok,
        ),
    }
}

// ---- Helpers ----------------------------------------------------

fn open(global: &GlobalFlags) -> Result<Box<dyn CanBackend>> {
    open_backend(
        map_interface(global.interface),
        global.channel.as_deref(),
        global.bitrate,
    )
    .map_err(|e| anyhow!("opening backend: {e}"))
}

/// Convert the CLI's `InterfaceType` enum into the transport-layer
/// equivalent. Same mapping the other subcommands use — promoted
/// from there to keep this file self-contained.
fn map_interface(i: InterfaceType) -> InterfaceType {
    // The transport layer's `open_backend` takes our InterfaceType
    // directly today; this indirection is a no-op but documents the
    // boundary in case the two types diverge in the future.
    i
}

/// Send the arm (or disarm) frame and wait for the ACK (`0x7F1` for AMS,
/// `0x7E1` for ECU) with the matching `enabled` flag. Returns `Err` on
/// timeout / wrong-flavour ACK / bus error.
async fn arm(backend: &dyn CanBackend, profile: Profile, enable: bool) -> Result<()> {
    // uDV arm is fire-and-forget: `0x7DE` sticky-enables the stream with no
    // ACK, and there's no disarm frame (the firmware clears it on reboot).
    if profile == Profile::Udv {
        if enable {
            backend
                .send(udv::build_arm_frame())
                .await
                .map_err(|e| anyhow!("sending uDV arm frame: {e}"))?;
        }
        return Ok(());
    }

    let arm_frame = match profile {
        Profile::Ams => build_arm_frame(enable),
        Profile::Ecu => ecu::build_arm_frame(enable),
        Profile::Udv => unreachable!("handled above"),
    };
    let ack_id = match profile {
        Profile::Ams => AMS_ACK_ID,
        Profile::Ecu => ecu::ECU_ACK_ID,
        Profile::Udv => unreachable!("handled above"),
    };

    backend
        .send(arm_frame)
        .await
        .map_err(|e| anyhow!("sending arm frame: {e}"))?;

    let started = Instant::now();
    while started.elapsed() < ACK_TIMEOUT {
        match backend.recv(POLL_TIMEOUT).await {
            Ok(frame) if frame.id == ack_id => {
                // Matching-flavour ACK ends the wait; a wrong-flavour ACK
                // (e.g. a stale `01` echo while disarming) is ignored so
                // we keep waiting for the one that matches.
                let acked = match profile {
                    Profile::Ams => {
                        matches!(decode_frame(&frame), Some(PitDiagFrame::Ack { enabled }) if enabled == enable)
                    }
                    Profile::Ecu => {
                        matches!(ecu::decode_frame(&frame), Some(ecu::EcuPitDiagFrame::Ack { enabled }) if enabled == enable)
                    }
                    Profile::Udv => unreachable!("handled above"),
                };
                if acked {
                    return Ok(());
                }
            }
            Ok(_) => {
                // Some other ID during the arm window — ignore.
            }
            Err(TransportError::Timeout(_)) => {
                // Quiet bus during the poll window. Keep waiting.
            }
            Err(err) => {
                return Err(anyhow!("waiting for ACK: {err}"));
            }
        }
    }
    Err(anyhow!(
        "no {} ACK from {} within {}ms — is it on the bus, with pit-diag firmware?",
        if enable { "enable" } else { "disable" },
        match profile {
            Profile::Ams => "AMS",
            Profile::Ecu => "ECU",
            Profile::Udv => unreachable!("handled above"),
        },
        ACK_TIMEOUT.as_millis()
    ))
}

#[cfg(test)]
mod tests {
    use super::{parse_profile, Profile};

    #[test]
    fn parse_profile_accepts_ams_ecu_udv() {
        assert_eq!(parse_profile("ams").unwrap(), Profile::Ams);
        assert_eq!(parse_profile("ecu").unwrap(), Profile::Ecu);
        assert_eq!(parse_profile("udv").unwrap(), Profile::Udv);
    }

    #[test]
    fn parse_profile_rejects_unknown() {
        let err = parse_profile("xyz").unwrap_err().to_string();
        assert!(err.contains("xyz"), "message names the bad profile: {err}");
        assert!(parse_profile("").is_err());
    }

    // Wrap the subcommand enum so clap can parse it standalone in-test.
    #[derive(clap::Parser)]
    struct TestCli {
        #[command(subcommand)]
        cmd: super::PitDiagCommand,
    }

    #[test]
    fn listen_defaults_to_all_until_ctrl_c() {
        use clap::Parser;
        let cli = TestCli::try_parse_from(["x", "listen"]).unwrap();
        match cli.cmd {
            super::PitDiagCommand::Listen(a) => {
                // `all` is the default — a passive listen wants any board's
                // ungated health (ECU 0x704 / AMS 0x6CA).
                assert_eq!(a.profile, "all");
                // No duration → listen until Ctrl-C (health-light mode).
                assert_eq!(a.duration_ms, None);
            }
            _ => panic!("expected Listen"),
        }
    }

    #[test]
    fn listen_accepts_profile_and_duration() {
        use clap::Parser;
        let cli =
            TestCli::try_parse_from(["x", "listen", "--profile", "ams", "--duration-ms", "1500"])
                .unwrap();
        match cli.cmd {
            super::PitDiagCommand::Listen(a) => {
                assert_eq!(a.profile, "ams");
                assert_eq!(a.duration_ms, Some(1500));
            }
            _ => panic!("expected Listen"),
        }
    }
}
