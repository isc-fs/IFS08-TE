//! ISO-TP multi-frame transport.
//!
//! Classic CAN (ISO 11898-1) carries at most 8 bytes per frame.
//! Bootloader messages routinely exceed that — a `GET_HEALTH` reply is
//! 33 bytes, a `GET_FW_INFO` reply is 65 bytes, a `DTC_READ` reply can
//! be up to 642 bytes. ISO-TP (ISO 15765-2) fragments a larger message
//! across multiple frames and reassembles it on the far side.
//!
//! ## Frame kinds (byte 0 high nibble)
//!
//! | PCI | Nibble | Layout                                                           |
//! |-----|--------|------------------------------------------------------------------|
//! | SF  | `0x0L` | `L` = payload length 1..7; bytes 1..L+1 = payload                |
//! | FF  | `0x1H` | `H` = high nibble of 12-bit total length; byte 1 = low byte;    |
//! |     |        | bytes 2..8 = first 6 bytes of payload                            |
//! | CF  | `0x2N` | `N` = sequence number 0..15 (wraps); bytes 1..8 = payload        |
//! | FC  | `0x3S` | `S` = 0 CTS / 1 Wait / 2 Overflow; byte 1 = BlockSize;           |
//! |     |        | byte 2 = SeparationTime                                          |
//!
//! The bootloader's ISO-TP convention on this bus:
//! - FF keeps the original message TYPE (CMD / NOTIFY / ACK / NACK / …).
//! - CFs and the receiver's FC reply use TYPE = DATA.
//!
//! This module handles the PCI and payload layout **inside** each
//! 8-byte CAN frame. The CAN ID (including the message TYPE
//! convention) is the caller's responsibility — the segmenter returns
//! raw 8-byte payloads, the reassembler takes raw 8-byte payloads.
//! Keeping this module ID-agnostic means adapter code can reuse it
//! without pulling in the whole `ids.rs` enum surface.
//!
//! ## Scope
//!
//! **Implemented** (v1):
//! - TX segmentation (SF; FF+CF for `1 <= len <= MAX_MSG_LEN`).
//! - RX reassembly with first-CF seq = 1, wrapping at 16 per ISO
//!   15765.
//! - 1024-byte max message length, matching `BL_ISOTP_MAX_MSG`.
//! - `tick` method that returns `Err(Timeout)` once elapsed since the
//!   last received frame exceeds the configured bound. Matches the
//!   bootloader's collapsed-timer approach (one total timeout rather
//!   than per-stage N_Bs / N_Cr).
//!
//! **Not implemented** (doesn't appear on the bus today):
//! - 32-bit escape FF (messages > 4095 bytes).
//! - FC(Wait) backpressure during RX.
//! - Sender-side FC handling (segmenter ignores received FC frames —
//!   we send every CF back-to-back and rely on the bootloader's
//!   advertised BS=0, STmin=0 defaults).
//! - Parallel / interleaved reassemblies against multiple peers.

use super::ParseError;

// ---- Spec constants (mirror `bl_isotp.h`) ----

/// Max reassembled message length, in bytes. Matches
/// `BL_ISOTP_MAX_MSG`.
pub const MAX_MSG_LEN: usize = 1024;

/// Payload bytes in a single SF (byte 0 = PCI, bytes 1..=7 = payload).
pub const SF_MAX_PAYLOAD: usize = 7;

/// Payload bytes in a FF (bytes 0..=1 = PCI+length, bytes 2..=7 = payload).
pub const FF_INITIAL_PAYLOAD: usize = 6;

/// Payload bytes in a CF (byte 0 = PCI+seq, bytes 1..=7 = payload).
pub const CF_PAYLOAD: usize = 7;

/// Every ISO-TP frame on the wire is exactly 8 bytes; short payloads
/// are zero-padded up to this width.
pub const FRAME_LEN: usize = 8;

// ---- PCI byte layout ----

const PCI_MASK_HI: u8 = 0xF0;
const PCI_MASK_LO: u8 = 0x0F;

const PCI_SF: u8 = 0x00;
const PCI_FF: u8 = 0x10;
const PCI_CF: u8 = 0x20;
const PCI_FC: u8 = 0x30;

// ---- FC status codes ----

/// Flow-control "clear to send" — continue transmitting CFs.
pub const FC_CTS: u8 = 0x00;
/// Flow-control "wait" — hold off and wait for another FC.
pub const FC_WAIT: u8 = 0x01;
/// Flow-control "overflow" — receiver can't fit the declared length.
pub const FC_OVERFLOW: u8 = 0x02;

/// Default block size the bootloader advertises in its FC frames.
/// `0` means "send every CF without requiring another FC".
pub const FC_BS_DEFAULT: u8 = 0x00;

/// Default STmin the bootloader advertises in its FC frames.
/// `0` means "no minimum inter-CF gap".
pub const FC_STMIN_DEFAULT: u8 = 0x00;

// ---- Errors ----

/// Every way ISO-TP reassembly / framing can go wrong.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum IsoTpError {
    /// PCI byte doesn't match any known frame kind, or a frame is
    /// shorter than its PCI requires (e.g. FF with only one byte).
    #[error("malformed PCI byte or frame length")]
    BadPci,

    /// A CF arrived with a sequence number that doesn't match the
    /// expected one.
    #[error("unexpected CF sequence: expected {expected}, got {got}")]
    BadSeq { expected: u8, got: u8 },

    /// FF declared a total length bigger than [`MAX_MSG_LEN`].
    #[error("declared length {declared} exceeds max {max}")]
    Overflow { declared: usize, max: usize },

    /// A CF arrived when no FF was active (reassembler is idle).
    #[error("consecutive frame with no first frame active")]
    NoFirstFrame,

    /// Reassembly didn't complete within the configured timeout. The
    /// caller feeds `tick` the current monotonic clock; this is the
    /// outcome of that tick rather than a frame feed.
    #[error("reassembly timed out after {elapsed_ms}ms")]
    Timeout { elapsed_ms: u64 },
}

// ---- TX: stateless segmenter ----

/// Iterator over ISO-TP-framed 8-byte payloads for a single message.
///
/// For a payload of length `L`:
/// - `L <= 7`: one SF frame.
/// - `L == 0`: **still one SF frame**, but with length nibble 0. The
///   bootloader doesn't send zero-length payloads, but the segmenter
///   accepts them so "no args" commands stay symmetrical.
/// - `L <= 4095`: one FF + `ceil((L - 6) / 7)` CFs, seq=1..wrap mod 16.
///
/// The iterator yields 8-byte `[u8; FRAME_LEN]` buffers; unused tail
/// bytes within each frame are zero-padded.
#[derive(Debug)]
pub struct IsoTpSegmenter<'a> {
    payload: &'a [u8],
    offset: usize,
    next_seq: u8,
    emitted_first: bool,
}

impl<'a> IsoTpSegmenter<'a> {
    /// Start segmenting `payload`. Returns an error if the payload is
    /// longer than [`MAX_MSG_LEN`] — a caller who knew better could
    /// try to emit a FF with a bigger declared length, and we don't
    /// want to silently truncate.
    pub fn new(payload: &'a [u8]) -> Result<Self, IsoTpError> {
        if payload.len() > MAX_MSG_LEN {
            return Err(IsoTpError::Overflow {
                declared: payload.len(),
                max: MAX_MSG_LEN,
            });
        }
        Ok(Self {
            payload,
            offset: 0,
            next_seq: 1, // first CF carries seq=1
            emitted_first: false,
        })
    }

    /// Total frames this segmenter will emit. Useful for progress
    /// reporting (`indicatif` bars during flash write).
    pub fn frame_count(&self) -> usize {
        let len = self.payload.len();
        if len <= SF_MAX_PAYLOAD {
            1
        } else {
            let after_ff = len - FF_INITIAL_PAYLOAD;
            let cf_count = after_ff.div_ceil(CF_PAYLOAD);
            1 + cf_count
        }
    }
}

impl Iterator for IsoTpSegmenter<'_> {
    type Item = [u8; FRAME_LEN];

    fn next(&mut self) -> Option<[u8; FRAME_LEN]> {
        let len = self.payload.len();

        // Case 1: short payload → one SF, then done.
        if !self.emitted_first && len <= SF_MAX_PAYLOAD {
            let mut frame = [0u8; FRAME_LEN];
            frame[0] = PCI_SF | (len as u8 & PCI_MASK_LO);
            frame[1..1 + len].copy_from_slice(&self.payload[..len]);
            self.emitted_first = true;
            self.offset = len;
            return Some(frame);
        }

        // Case 2: longer payload → FF first, then CFs.
        if !self.emitted_first {
            let total_len = len as u16;
            debug_assert!(total_len <= 0x0FFF);
            let mut frame = [0u8; FRAME_LEN];
            frame[0] = PCI_FF | ((total_len >> 8) as u8 & PCI_MASK_LO);
            frame[1] = total_len as u8;
            frame[2..FRAME_LEN].copy_from_slice(&self.payload[..FF_INITIAL_PAYLOAD]);
            self.emitted_first = true;
            self.offset = FF_INITIAL_PAYLOAD;
            return Some(frame);
        }

        // Case 3: keep emitting CFs until we run out of payload.
        if self.offset >= len {
            return None;
        }

        let mut frame = [0u8; FRAME_LEN];
        frame[0] = PCI_CF | (self.next_seq & PCI_MASK_LO);
        let remaining = len - self.offset;
        let take = remaining.min(CF_PAYLOAD);
        frame[1..1 + take].copy_from_slice(&self.payload[self.offset..self.offset + take]);
        self.offset += take;
        self.next_seq = (self.next_seq + 1) & 0x0F;
        Some(frame)
    }
}

/// Build a flow-control frame. Caller owns the CAN ID construction
/// (TYPE=DATA, dst=peer) — this function only produces the 8-byte
/// payload.
pub fn build_fc(status: u8, block_size: u8, st_min: u8) -> [u8; FRAME_LEN] {
    let mut frame = [0u8; FRAME_LEN];
    frame[0] = PCI_FC | (status & PCI_MASK_LO);
    frame[1] = block_size;
    frame[2] = st_min;
    frame
}

/// Convenience: build the FC(CTS, BS=0, STmin=0) the bootloader expects.
pub fn build_fc_cts() -> [u8; FRAME_LEN] {
    build_fc(FC_CTS, FC_BS_DEFAULT, FC_STMIN_DEFAULT)
}

// ---- RX: stateful reassembler ----

/// One of three completion states returned by
/// [`Reassembler::feed`] / [`Reassembler::tick`].
#[derive(Debug)]
pub enum ReassembleOutcome {
    /// Frame was consumed; reassembly is ongoing but not yet complete.
    Ongoing,
    /// Reassembly completed — message payload returned by value.
    Complete(Vec<u8>),
}

/// Internal state of the reassembler.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    WaitCf,
}

/// RX reassembler. One instance handles **one** in-flight reassembly
/// at a time — matches the bootloader's single-reassembly policy.
/// Callers with multiple peers open a `Reassembler` per peer.
///
/// Timing: the reassembler remembers the millisecond timestamp of the
/// last frame it accepted (supplied by the caller via
/// [`Reassembler::feed`]); [`Reassembler::tick`] uses that plus the
/// configured timeout to decide whether to emit `Err(Timeout)`. We
/// don't pull in `std::time::Instant` because we want this module to
/// be portable (no-std-friendly) and trivially mockable in tests.
#[derive(Debug)]
pub struct Reassembler {
    state: State,
    buffer: Vec<u8>,
    total_len: usize,
    expected_seq: u8,
    last_rx_ms: u64,
    timeout_ms: u64,
}

impl Reassembler {
    /// Default total-reassembly timeout: 1000 ms. Matches
    /// `BL_ISOTP_TIMEOUT_MS`.
    pub const DEFAULT_TIMEOUT_MS: u64 = 1000;

    /// Create an idle reassembler with the default timeout.
    pub fn new() -> Self {
        Self::with_timeout(Self::DEFAULT_TIMEOUT_MS)
    }

    /// Create an idle reassembler with a custom timeout.
    pub fn with_timeout(timeout_ms: u64) -> Self {
        Self {
            state: State::Idle,
            buffer: Vec::new(),
            total_len: 0,
            expected_seq: 0,
            last_rx_ms: 0,
            timeout_ms,
        }
    }

    /// True when the reassembler is waiting for the first frame of a
    /// message. False once a FF has been accepted but the reassembly
    /// isn't complete yet.
    pub fn is_idle(&self) -> bool {
        self.state == State::Idle
    }

    /// Bytes buffered so far in the current reassembly, or `0` when
    /// idle. Intended for live-data / progress reporting.
    pub fn progress(&self) -> usize {
        self.buffer.len()
    }

    /// Reset back to idle. Drops any buffered bytes. Callers invoke
    /// this after a NACK(TRANSPORT_*) escalation, a session
    /// teardown, or anywhere else the current reassembly has to be
    /// abandoned.
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.buffer.clear();
        self.total_len = 0;
        self.expected_seq = 0;
    }

    /// Feed one 8-byte ISO-TP frame payload (the bytes inside the
    /// CAN frame, not the CAN frame itself). `now_ms` is the
    /// caller's monotonic clock, used for timeout tracking.
    ///
    /// Returns:
    /// - `Ok(Ongoing)` — frame consumed, reassembly not yet complete.
    /// - `Ok(Complete(bytes))` — reassembly finished; `bytes` is the
    ///   full message payload. Reassembler is reset to `Idle`.
    /// - `Err(error)` — frame was malformed; reassembler is reset to
    ///   `Idle` so the next FF starts fresh.
    pub fn feed(
        &mut self,
        frame_payload: &[u8],
        now_ms: u64,
    ) -> Result<ReassembleOutcome, IsoTpError> {
        if frame_payload.is_empty() {
            self.reset();
            return Err(IsoTpError::BadPci);
        }
        let pci_hi = frame_payload[0] & PCI_MASK_HI;
        let pci_lo = frame_payload[0] & PCI_MASK_LO;

        match self.state {
            State::Idle => match pci_hi {
                PCI_SF => self.feed_sf(frame_payload, pci_lo, now_ms),
                PCI_FF => self.feed_ff(frame_payload, pci_lo, now_ms),
                PCI_CF => Err(IsoTpError::NoFirstFrame),
                PCI_FC => {
                    // Silently ignore FC while idle — segmenter
                    // doesn't react to FC today, and RX state won't
                    // either.
                    Ok(ReassembleOutcome::Ongoing)
                }
                _ => Err(IsoTpError::BadPci),
            },
            State::WaitCf => match pci_hi {
                PCI_CF => self.feed_cf(frame_payload, pci_lo, now_ms),
                // SF / FF / FC mid-reassembly means the peer restarted
                // the conversation. Reset and reinterpret the current
                // frame from Idle.
                _ => {
                    self.reset();
                    self.feed(frame_payload, now_ms)
                }
            },
        }
    }

    /// Update the timeout bookkeeping. Call periodically from the
    /// event loop. Returns `Err(Timeout)` when the last frame was
    /// accepted more than `timeout_ms` ago and the reassembler is
    /// mid-message; the reassembler resets to `Idle` before
    /// returning so the caller can treat this as fatal-for-the-msg
    /// but recoverable-for-the-session.
    pub fn tick(&mut self, now_ms: u64) -> Result<(), IsoTpError> {
        if self.state == State::Idle {
            return Ok(());
        }
        let elapsed = now_ms.saturating_sub(self.last_rx_ms);
        if elapsed > self.timeout_ms {
            self.reset();
            return Err(IsoTpError::Timeout {
                elapsed_ms: elapsed,
            });
        }
        Ok(())
    }

    // ---- Internals ----

    fn feed_sf(
        &mut self,
        frame: &[u8],
        pci_lo: u8,
        now_ms: u64,
    ) -> Result<ReassembleOutcome, IsoTpError> {
        let len = pci_lo as usize;
        if len > SF_MAX_PAYLOAD || frame.len() < 1 + len {
            return Err(IsoTpError::BadPci);
        }
        let out = frame[1..1 + len].to_vec();
        self.last_rx_ms = now_ms;
        // Stay idle; SF is complete in one frame.
        self.state = State::Idle;
        Ok(ReassembleOutcome::Complete(out))
    }

    fn feed_ff(
        &mut self,
        frame: &[u8],
        pci_lo: u8,
        now_ms: u64,
    ) -> Result<ReassembleOutcome, IsoTpError> {
        if frame.len() < 2 {
            return Err(IsoTpError::BadPci);
        }
        let total_len = (usize::from(pci_lo) << 8) | usize::from(frame[1]);
        if total_len <= SF_MAX_PAYLOAD {
            // FF with a length that would fit in an SF is ill-formed.
            return Err(IsoTpError::BadPci);
        }
        if total_len > MAX_MSG_LEN {
            return Err(IsoTpError::Overflow {
                declared: total_len,
                max: MAX_MSG_LEN,
            });
        }
        if frame.len() < 2 + FF_INITIAL_PAYLOAD {
            return Err(IsoTpError::BadPci);
        }

        self.buffer.clear();
        self.buffer.reserve(total_len);
        self.buffer
            .extend_from_slice(&frame[2..2 + FF_INITIAL_PAYLOAD]);
        self.total_len = total_len;
        self.expected_seq = 1; // first CF carries seq=1
        self.last_rx_ms = now_ms;
        self.state = State::WaitCf;
        Ok(ReassembleOutcome::Ongoing)
    }

    fn feed_cf(
        &mut self,
        frame: &[u8],
        pci_lo: u8,
        now_ms: u64,
    ) -> Result<ReassembleOutcome, IsoTpError> {
        if pci_lo != self.expected_seq {
            let expected = self.expected_seq;
            self.reset();
            return Err(IsoTpError::BadSeq {
                expected,
                got: pci_lo,
            });
        }

        let remaining = self.total_len - self.buffer.len();
        let take = remaining.min(CF_PAYLOAD);
        if frame.len() < 1 + take {
            return Err(IsoTpError::BadPci);
        }
        self.buffer.extend_from_slice(&frame[1..1 + take]);

        self.expected_seq = (self.expected_seq + 1) & 0x0F;
        self.last_rx_ms = now_ms;

        if self.buffer.len() >= self.total_len {
            let done = std::mem::take(&mut self.buffer);
            self.state = State::Idle;
            self.total_len = 0;
            self.expected_seq = 0;
            Ok(ReassembleOutcome::Complete(done))
        } else {
            Ok(ReassembleOutcome::Ongoing)
        }
    }
}

impl Default for Reassembler {
    fn default() -> Self {
        Self::new()
    }
}

// Convenience: the module can flag a bare `ParseError` in its
// response-layer hot path, but ISO-TP callers deal in `IsoTpError`.
// The conversion is an escape hatch.
impl From<IsoTpError> for ParseError {
    fn from(err: IsoTpError) -> Self {
        match err {
            IsoTpError::BadPci => ParseError::Invalid("iso-tp: bad PCI"),
            IsoTpError::BadSeq { .. } => ParseError::Invalid("iso-tp: bad CF sequence"),
            IsoTpError::Overflow { .. } => ParseError::Invalid("iso-tp: overflow"),
            IsoTpError::NoFirstFrame => ParseError::Invalid("iso-tp: CF with no active FF"),
            IsoTpError::Timeout { .. } => ParseError::Invalid("iso-tp: reassembly timeout"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Segmenter ----

    #[test]
    fn segmenter_single_short_payload_emits_one_sf() {
        let seg = IsoTpSegmenter::new(&[0x01, 0xAA, 0xBB]).unwrap();
        assert_eq!(seg.frame_count(), 1);
        let frames: Vec<_> = seg.collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0][0], 0x03); // SF, len=3
        assert_eq!(&frames[0][1..4], &[0x01, 0xAA, 0xBB]);
        // Padding is zero.
        assert_eq!(&frames[0][4..], &[0u8; 4]);
    }

    #[test]
    fn segmenter_seven_byte_payload_fits_in_sf() {
        let payload: Vec<u8> = (0..7).collect();
        let frames: Vec<_> = IsoTpSegmenter::new(&payload).unwrap().collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0][0], 0x07);
        assert_eq!(&frames[0][1..8], &payload[..]);
    }

    #[test]
    fn segmenter_eight_byte_payload_needs_ff_plus_cf() {
        let payload: Vec<u8> = (0..8).collect();
        let seg = IsoTpSegmenter::new(&payload).unwrap();
        assert_eq!(seg.frame_count(), 2);
        let frames: Vec<_> = seg.collect();

        // FF: PCI high = 1, low = 0x0 (len 0x008 -> hi nibble 0); byte 1 = 0x08.
        assert_eq!(frames[0][0], 0x10);
        assert_eq!(frames[0][1], 0x08);
        assert_eq!(&frames[0][2..8], &payload[0..6]);

        // CF: seq=1, 2 bytes of payload, rest zero-pad.
        assert_eq!(frames[1][0], 0x21);
        assert_eq!(&frames[1][1..3], &payload[6..8]);
        assert_eq!(&frames[1][3..], &[0u8; 5]);
    }

    #[test]
    fn segmenter_cf_seq_wraps_after_15() {
        // Payload length 6 + 15*7 + 5 = 116 → FF + 16 CFs (last one partial).
        let payload: Vec<u8> = (0..116).map(|n| n as u8).collect();
        let frames: Vec<_> = IsoTpSegmenter::new(&payload).unwrap().collect();
        assert_eq!(frames.len(), 1 + 16);
        let cf_seqs: Vec<u8> = frames[1..].iter().map(|f| f[0] & 0x0F).collect();
        assert_eq!(
            cf_seqs,
            vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0]
        );
    }

    #[test]
    fn segmenter_rejects_oversize_payload() {
        let payload = vec![0u8; MAX_MSG_LEN + 1];
        assert!(matches!(
            IsoTpSegmenter::new(&payload),
            Err(IsoTpError::Overflow { .. })
        ));
    }

    // ---- Reassembler ----

    fn feed_all(
        reasm: &mut Reassembler,
        frames: &[[u8; FRAME_LEN]],
    ) -> Result<Vec<u8>, IsoTpError> {
        for (i, frame) in frames.iter().enumerate() {
            let out = reasm.feed(frame, (i as u64 + 1) * 10)?;
            match out {
                ReassembleOutcome::Complete(bytes) => return Ok(bytes),
                ReassembleOutcome::Ongoing => continue,
            }
        }
        Err(IsoTpError::BadPci) // never completed
    }

    #[test]
    fn reassembler_sf_roundtrip() {
        let payload: Vec<u8> = (0..5).collect();
        let frames: Vec<_> = IsoTpSegmenter::new(&payload).unwrap().collect();
        let mut rx = Reassembler::new();
        let got = feed_all(&mut rx, &frames).unwrap();
        assert_eq!(got, payload);
        assert!(rx.is_idle());
    }

    #[test]
    fn reassembler_ff_cf_roundtrip_exact() {
        // Exactly one CF worth of payload after the FF's 6 bytes.
        let payload: Vec<u8> = (0..13).collect();
        let frames: Vec<_> = IsoTpSegmenter::new(&payload).unwrap().collect();
        assert_eq!(frames.len(), 2);
        let mut rx = Reassembler::new();
        let got = feed_all(&mut rx, &frames).unwrap();
        assert_eq!(got, payload);
        assert!(rx.is_idle());
    }

    #[test]
    fn reassembler_ff_cf_roundtrip_large() {
        // Near the maximum.
        let payload: Vec<u8> = (0..1000).map(|n| (n & 0xFF) as u8).collect();
        let frames: Vec<_> = IsoTpSegmenter::new(&payload).unwrap().collect();
        let mut rx = Reassembler::new();
        let got = feed_all(&mut rx, &frames).unwrap();
        assert_eq!(got, payload);
    }

    #[test]
    fn reassembler_rejects_cf_without_ff() {
        let mut rx = Reassembler::new();
        let bogus_cf = [0x21, 0xAA, 0, 0, 0, 0, 0, 0];
        let err = rx.feed(&bogus_cf, 10).unwrap_err();
        assert_eq!(err, IsoTpError::NoFirstFrame);
        assert!(rx.is_idle());
    }

    #[test]
    fn reassembler_rejects_bad_seq() {
        // FF declaring 14 bytes, then a CF with the wrong seq.
        let ff = [0x10, 0x0E, 1, 2, 3, 4, 5, 6];
        let wrong_cf = [0x23, 7, 8, 9, 10, 11, 12, 13]; // seq=3, expected 1
        let mut rx = Reassembler::new();
        assert!(matches!(rx.feed(&ff, 10), Ok(ReassembleOutcome::Ongoing)));
        let err = rx.feed(&wrong_cf, 20).unwrap_err();
        assert!(matches!(
            err,
            IsoTpError::BadSeq {
                expected: 1,
                got: 3
            }
        ));
        assert!(rx.is_idle(), "reassembler should reset after bad seq");
    }

    #[test]
    fn reassembler_rejects_ff_declaring_sf_sized_payload() {
        let ff = [0x10, 0x05, 1, 2, 3, 4, 5, 6];
        let mut rx = Reassembler::new();
        assert!(matches!(rx.feed(&ff, 10), Err(IsoTpError::BadPci)));
    }

    #[test]
    fn reassembler_rejects_overflow_on_ff() {
        // 0x1_0001 = MAX_MSG_LEN+1 would need a 13-bit length, so use
        // the max 4095 representable here and confirm reasm rejects it
        // because it exceeds MAX_MSG_LEN (1024).
        let ff = [0x1F, 0xFF, 1, 2, 3, 4, 5, 6]; // declared = 0xFFF = 4095
        let mut rx = Reassembler::new();
        let err = rx.feed(&ff, 10).unwrap_err();
        assert!(matches!(
            err,
            IsoTpError::Overflow {
                declared: 4095,
                max: 1024
            }
        ));
    }

    #[test]
    fn reassembler_tick_fires_after_timeout() {
        let ff = [0x10, 0x0E, 1, 2, 3, 4, 5, 6];
        let mut rx = Reassembler::with_timeout(500);
        rx.feed(&ff, 10).unwrap();
        assert!(rx.tick(400).is_ok());
        assert!(rx.tick(509).is_ok()); // 509 - 10 = 499, under threshold
        let err = rx.tick(1200).unwrap_err();
        assert!(matches!(err, IsoTpError::Timeout { .. }));
        assert!(rx.is_idle());
    }

    #[test]
    fn reassembler_restart_on_new_ff_mid_flight() {
        // The first FF arrives, then a fresh FF from the same peer —
        // the reassembler should discard the half-built message and
        // restart on the new one.
        let ff_1 = [0x10, 0x0E, 1, 2, 3, 4, 5, 6];
        let ff_2 = [0x10, 0x08, 9, 9, 9, 9, 9, 9];
        let cf_2 = [0x21, 8, 8, 0, 0, 0, 0, 0]; // 2 bytes after FF's 6 = 8 total
        let mut rx = Reassembler::new();
        assert!(matches!(rx.feed(&ff_1, 10), Ok(ReassembleOutcome::Ongoing)));
        assert!(matches!(rx.feed(&ff_2, 20), Ok(ReassembleOutcome::Ongoing)));
        match rx.feed(&cf_2, 30) {
            Ok(ReassembleOutcome::Complete(bytes)) => {
                assert_eq!(bytes, vec![9, 9, 9, 9, 9, 9, 8, 8]);
            }
            other => panic!("expected Complete, got {other:?}"),
        }
    }

    #[test]
    fn build_fc_cts_has_expected_shape() {
        let fc = build_fc_cts();
        assert_eq!(fc[0], 0x30);
        assert_eq!(fc[1], 0x00);
        assert_eq!(fc[2], 0x00);
        assert_eq!(&fc[3..], &[0u8; 5]);
    }

    // ---- Bootloader v1.2.0 strict-reassembler simulation ----
    //
    // The bootloader (per `bl_isotp.c` in stm32-can-bootloader@main)
    // added two reject conditions in v1.2.0 (PR #63 + #78) that the
    // pre-v1.2 BL silently dropped:
    //
    //   1. PCI nibble 4..F (in either Idle or WaitCf state)
    //      → `BL_ISOTP_ERR_BAD_PCI`
    //   2. FF with CAN DLC < 8       → `BL_ISOTP_ERR_BAD_PCI`
    //   3. CF with CAN DLC < 2       → `BL_ISOTP_ERR_BAD_PCI`
    //   4. FF with `total_len == 0`  → `BL_ISOTP_ERR_BAD_PCI`
    //
    // These tests verify the host-side segmenter never emits a frame
    // stream that would trip any of these. Operator-facing context:
    // issue #178 reports a `NACK(TRANSPORT_ERROR)` on the first
    // FLASH_WRITE_CHUNK against a v1.2.0-rc1 bootloader — these
    // tests should fail if the segmenter has a bug that the new BL
    // strictness now catches.

    /// Mirrors the BL's `bl_isotp_rx_feed` dispatch + `handle_ff` +
    /// `handle_cf` reject rules. DLC of every frame is treated as
    /// the slice length passed in (caller is responsible for passing
    /// the actual on-wire byte count).
    fn bl_v1_2_strict_check(frames: &[&[u8]]) -> Result<(), IsoTpError> {
        let mut next_seq: u8 = 0;
        let mut total_len: usize = 0;
        let mut received: usize = 0;
        for frame in frames {
            if frame.is_empty() {
                return Err(IsoTpError::BadPci);
            }
            let pci_hi = frame[0] & PCI_MASK_HI;
            let pci_lo = frame[0] & PCI_MASK_LO;
            match pci_hi {
                PCI_SF => {
                    let len = pci_lo as usize;
                    if frame.len() < 1 + len {
                        return Err(IsoTpError::BadPci);
                    }
                }
                PCI_FF => {
                    if frame.len() < 8 {
                        return Err(IsoTpError::BadPci);
                    }
                    total_len = (usize::from(pci_lo) << 8) | usize::from(frame[1]);
                    if total_len == 0 {
                        return Err(IsoTpError::BadPci);
                    }
                    if total_len > MAX_MSG_LEN {
                        return Err(IsoTpError::Overflow {
                            declared: total_len,
                            max: MAX_MSG_LEN,
                        });
                    }
                    received = FF_INITIAL_PAYLOAD.min(total_len);
                    next_seq = 1;
                }
                PCI_CF => {
                    if frame.len() < 2 {
                        return Err(IsoTpError::BadPci);
                    }
                    if pci_lo != next_seq {
                        return Err(IsoTpError::BadSeq {
                            expected: next_seq,
                            got: pci_lo,
                        });
                    }
                    let remaining = total_len - received;
                    let frame_avail = frame.len() - 1;
                    let take = remaining.min(frame_avail);
                    received += take;
                    next_seq = (next_seq + 1) & 0x0F;
                }
                PCI_FC => {} // BL doesn't validate FC in rx-feed
                _ => return Err(IsoTpError::BadPci),
            }
        }
        Ok(())
    }

    /// Helper: the on-wire DLC of every CAN frame the flasher emits
    /// is always 8 (classic CAN). The segmenter yields `[u8; 8]` and
    /// the session/SocketCAN paths preserve that length through to
    /// the kernel. Treat each segmenter output as an 8-byte frame.
    fn as_full_frames(frames: &[[u8; FRAME_LEN]]) -> Vec<&[u8]> {
        frames.iter().map(|f| &f[..]).collect()
    }

    #[test]
    fn strict_bl_accepts_segmented_flash_write_chunk_262_bytes() {
        // Real flasher payload for one FLASH_WRITE_CHUNK with the new
        // wire format: msg_type (1) + opcode (1) + addr (4) + data (256).
        let payload: Vec<u8> = (0..262).map(|i| (i & 0xFF) as u8).collect();
        let frames: Vec<[u8; FRAME_LEN]> = IsoTpSegmenter::new(&payload).unwrap().collect();
        assert_eq!(frames.len(), 1 + 37, "262-byte payload = 1 FF + 37 CFs");
        let frame_slices = as_full_frames(&frames);
        bl_v1_2_strict_check(&frame_slices)
            .expect("segmenter output must pass BL v1.2.0 strict rules");
    }

    #[test]
    fn strict_bl_accepts_message_lengths_around_cf_boundaries() {
        // Edge: total_len - 6 exactly divisible by 7 (last CF is full).
        // Edge: total_len - 6 has remainder 1 (last CF carries 1 byte).
        // Edge: total_len exactly 8 (single CF of 2 bytes).
        for total_len in [
            8_usize, 13, 14, 20, 27, 64, 100, 128, 256, 261, 262, 519, 1024,
        ] {
            let payload: Vec<u8> = (0..total_len).map(|i| (i & 0xFF) as u8).collect();
            let frames: Vec<[u8; FRAME_LEN]> = IsoTpSegmenter::new(&payload).unwrap().collect();
            let frame_slices = as_full_frames(&frames);
            bl_v1_2_strict_check(&frame_slices).unwrap_or_else(|e| {
                panic!(
                    "BL v1.2 strict rules rejected segmenter output for \
                     total_len={total_len} (frames={}): {e:?}",
                    frames.len()
                )
            });
        }
    }

    #[test]
    fn strict_bl_rejects_ff_with_dlc_under_8() {
        // Defence-in-depth: confirm our strict checker catches the
        // BL's known reject conditions. If the flasher ever emits an
        // FF with DLC < 8 it would trip this rule and the test on
        // top would fail.
        let ff_short = [0x10_u8, 0x08, 1, 2, 3, 4]; // DLC=6
        assert!(matches!(
            bl_v1_2_strict_check(&[&ff_short[..]]),
            Err(IsoTpError::BadPci)
        ));
    }

    #[test]
    fn strict_bl_rejects_cf_with_dlc_under_2() {
        // FF declaring 10 bytes then a CF with only the PCI byte (DLC=1).
        let ff = [0x10_u8, 0x0A, 1, 2, 3, 4, 5, 6];
        let cf_short = [0x21_u8]; // DLC=1
        assert!(matches!(
            bl_v1_2_strict_check(&[&ff[..], &cf_short[..]]),
            Err(IsoTpError::BadPci)
        ));
    }
}
