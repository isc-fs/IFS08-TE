//! Fixed-layout record structs that ride inside ISO-TP-reassembled
//! message payloads.
//!
//! Each record documented here must match the `_Static_assert`'d
//! layout in the corresponding bootloader header:
//!
//! | Record               | Header            | Size |
//! |----------------------|-------------------|-----:|
//! | [`FirmwareInfo`]     | `bl_fwinfo.h`     | 64 B |
//! | [`HealthRecord`]     | `bl_health.h`     | 32 B |
//! | [`LiveDataSnapshot`] | `bl_live.h`       | 32 B |
//! | [`DtcEntry`]         | `bl_dtc.h`        | 20 B |
//! | [`ObStatus`]         | `bl_obyte.h`      | 16 B |
//!
//! All multi-byte fields are little-endian on the wire.

use super::ParseError;

// ---- Helpers for LE parsing ----

fn read_u16_le(bytes: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([bytes[off], bytes[off + 1]])
}

fn read_u32_le(bytes: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]])
}

fn read_u64_le(bytes: &[u8], off: usize) -> u64 {
    let mut buf = [0u8; 8];
    buf.copy_from_slice(&bytes[off..off + 8]);
    u64::from_le_bytes(buf)
}

fn ensure_len(bytes: &[u8], need: usize) -> Result<(), ParseError> {
    if bytes.len() < need {
        Err(ParseError::RecordTooShort {
            got: bytes.len(),
            need,
        })
    } else {
        Ok(())
    }
}

// ---- FirmwareInfo (64 B) ----

/// The `__firmware_info` record the application publishes at
/// `0x08020400`. The bootloader reads it (never writes it) and sends
/// it back in the ACK payload of `CMD_GET_FW_INFO`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FirmwareInfo {
    pub magic: u32,
    pub record_version: u32,
    pub fw_version_major: u32,
    pub fw_version_minor: u32,
    pub fw_version_patch: u32,
    pub mcu_id: u32,
    pub git_hash: [u8; 8],
    pub build_timestamp: u64,
    /// 16-byte NUL-padded ASCII; use [`FirmwareInfo::product_name`] to
    /// get a trimmed `&str`.
    pub product_name: [u8; 16],
    pub reserved: [u32; 2],
}

impl FirmwareInfo {
    /// On-wire size in bytes (matches `bl_fwinfo_t`).
    pub const SIZE: usize = 64;

    /// Expected `magic` value (`BL_FWINFO_MAGIC`).
    pub const MAGIC: u32 = 0xF14F1B00;

    /// Major version the host tool speaks. Records with a higher
    /// major version elicit `ParseError::UnsupportedRecordVersion`.
    pub const SUPPORTED_MAJOR: u16 = 1;

    /// Parse the 64-byte record. Validates the magic and major
    /// version; accepts any minor version (forward-compat).
    pub fn parse(bytes: &[u8]) -> Result<Self, ParseError> {
        ensure_len(bytes, Self::SIZE)?;
        let magic = read_u32_le(bytes, 0);
        if magic != Self::MAGIC {
            return Err(ParseError::MagicMismatch {
                got: magic,
                want: Self::MAGIC,
            });
        }
        let record_version = read_u32_le(bytes, 4);
        let major = (record_version >> 16) as u16;
        if major > Self::SUPPORTED_MAJOR {
            return Err(ParseError::UnsupportedRecordVersion {
                got: record_version,
                supported_major: Self::SUPPORTED_MAJOR,
            });
        }

        let mut git_hash = [0u8; 8];
        git_hash.copy_from_slice(&bytes[24..32]);
        let mut product_name = [0u8; 16];
        product_name.copy_from_slice(&bytes[40..56]);

        Ok(Self {
            magic,
            record_version,
            fw_version_major: read_u32_le(bytes, 8),
            fw_version_minor: read_u32_le(bytes, 12),
            fw_version_patch: read_u32_le(bytes, 16),
            mcu_id: read_u32_le(bytes, 20),
            git_hash,
            build_timestamp: read_u64_le(bytes, 32),
            product_name,
            reserved: [read_u32_le(bytes, 56), read_u32_le(bytes, 60)],
        })
    }

    /// Return the product name as a trimmed `&str` — everything up to
    /// the first NUL. Falls back to an empty string if the bytes
    /// aren't valid UTF-8.
    pub fn product_name(&self) -> &str {
        let end = self
            .product_name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.product_name.len());
        std::str::from_utf8(&self.product_name[..end]).unwrap_or("")
    }

    /// `(major, minor, patch)` tuple for display.
    pub fn version(&self) -> (u32, u32, u32) {
        (
            self.fw_version_major,
            self.fw_version_minor,
            self.fw_version_patch,
        )
    }
}

// ---- HealthRecord (32 B) ----

/// The 32-byte record returned by `CMD_GET_HEALTH`. See
/// `bl_health.h` for the on-device producer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HealthRecord {
    pub uptime_seconds: u32,
    pub reset_cause: u32,
    pub flags: u32,
    pub flash_write_count: u32,
    pub dtc_count: u32,
    pub last_dtc_code: u32,
    /// offset 24 — FDCAN bus-off Stop/Start recoveries this boot (#125 C1).
    /// Formerly reserved[0].
    pub fdcan_recovery_count: u32,
    /// offset 28 — longest blocking flash op (erase) observed this boot, ms
    /// (#125 H6 probe — sizes the IWDG period). Formerly reserved[1].
    pub max_flash_op_ms: u32,
}

impl HealthRecord {
    pub const SIZE: usize = 32;

    pub fn parse(bytes: &[u8]) -> Result<Self, ParseError> {
        ensure_len(bytes, Self::SIZE)?;
        Ok(Self {
            uptime_seconds: read_u32_le(bytes, 0),
            reset_cause: read_u32_le(bytes, 4),
            flags: read_u32_le(bytes, 8),
            flash_write_count: read_u32_le(bytes, 12),
            dtc_count: read_u32_le(bytes, 16),
            last_dtc_code: read_u32_le(bytes, 20),
            fdcan_recovery_count: read_u32_le(bytes, 24),
            max_flash_op_ms: read_u32_le(bytes, 28),
        })
    }

    /// True if `BL_HEALTH_FLAG_SESSION_ACTIVE` is set.
    pub fn session_active(&self) -> bool {
        self.flags & HEALTH_FLAG_SESSION_ACTIVE != 0
    }

    /// True if `BL_HEALTH_FLAG_VALID_APP_PRESENT` is set.
    pub fn valid_app_present(&self) -> bool {
        self.flags & HEALTH_FLAG_VALID_APP_PRESENT != 0
    }

    /// True if `BL_HEALTH_FLAG_WRP_PROTECTED` is set (sector 0 is
    /// latched behind WRP).
    pub fn wrp_protected(&self) -> bool {
        self.flags & HEALTH_FLAG_WRP_PROTECTED != 0
    }

    /// Decoded reset cause, or `None` for unknown values.
    pub fn reset_cause(&self) -> Option<ResetCause> {
        ResetCause::from_u32(self.reset_cause)
    }
}

pub const HEALTH_FLAG_SESSION_ACTIVE: u32 = 1 << 0;
pub const HEALTH_FLAG_VALID_APP_PRESENT: u32 = 1 << 1;
pub const HEALTH_FLAG_WRP_PROTECTED: u32 = 1 << 4;

// ---- ResetCause (one byte carried in heartbeat; one u32 field in
// HealthRecord) ----

/// Values match `BL_RESET_*` in `bl_health.h`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ResetCause {
    Unknown = 0x00,
    PowerOn = 0x01,
    Pin = 0x02,
    Software = 0x03,
    Iwdg = 0x04,
    Wwdg = 0x05,
    LowPower = 0x06,
    Brownout = 0x07,
}

impl ResetCause {
    pub fn from_u32(value: u32) -> Option<Self> {
        Self::from_byte(value as u8)
    }

    pub fn from_byte(byte: u8) -> Option<Self> {
        Some(match byte {
            0x00 => Self::Unknown,
            0x01 => Self::PowerOn,
            0x02 => Self::Pin,
            0x03 => Self::Software,
            0x04 => Self::Iwdg,
            0x05 => Self::Wwdg,
            0x06 => Self::LowPower,
            0x07 => Self::Brownout,
            _ => return None,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unknown => "UNKNOWN",
            Self::PowerOn => "POWER_ON",
            Self::Pin => "PIN",
            Self::Software => "SOFTWARE",
            Self::Iwdg => "IWDG",
            Self::Wwdg => "WWDG",
            Self::LowPower => "LOW_POWER",
            Self::Brownout => "BROWNOUT",
        }
    }
}

// ---- LiveDataSnapshot (32 B) ----

/// 32-byte snapshot emitted by `NOTIFY_LIVE_DATA`. See `bl_live.h`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LiveDataSnapshot {
    pub uptime_ms: u32,
    pub frames_rx: u16,
    pub frames_tx: u16,
    pub nacks_sent: u16,
    pub dtc_count: u16,
    pub last_dtc_code: u16,
    pub flags: u8,
    pub last_opcode: u8,
    pub last_flash_addr: u32,
    pub isotp_rx_progress: u32,
    pub session_age_ms: u32,
    pub reserved: u32,
}

impl LiveDataSnapshot {
    pub const SIZE: usize = 32;

    pub fn parse(bytes: &[u8]) -> Result<Self, ParseError> {
        ensure_len(bytes, Self::SIZE)?;
        Ok(Self {
            uptime_ms: read_u32_le(bytes, 0),
            frames_rx: read_u16_le(bytes, 4),
            frames_tx: read_u16_le(bytes, 6),
            nacks_sent: read_u16_le(bytes, 8),
            dtc_count: read_u16_le(bytes, 10),
            last_dtc_code: read_u16_le(bytes, 12),
            flags: bytes[14],
            last_opcode: bytes[15],
            last_flash_addr: read_u32_le(bytes, 16),
            isotp_rx_progress: read_u32_le(bytes, 20),
            session_age_ms: read_u32_le(bytes, 24),
            reserved: read_u32_le(bytes, 28),
        })
    }

    pub fn session_active(&self) -> bool {
        self.flags & LIVE_FLAG_SESSION_ACTIVE != 0
    }

    pub fn valid_app_present(&self) -> bool {
        self.flags & LIVE_FLAG_VALID_APP_PRESENT != 0
    }

    pub fn log_streaming(&self) -> bool {
        self.flags & LIVE_FLAG_LOG_STREAMING != 0
    }

    pub fn livedata_streaming(&self) -> bool {
        self.flags & LIVE_FLAG_LIVEDATA_STREAMING != 0
    }

    pub fn wrp_protected(&self) -> bool {
        self.flags & LIVE_FLAG_WRP_PROTECTED != 0
    }
}

pub const LIVE_FLAG_SESSION_ACTIVE: u8 = 1 << 0;
pub const LIVE_FLAG_VALID_APP_PRESENT: u8 = 1 << 1;
pub const LIVE_FLAG_LOG_STREAMING: u8 = 1 << 2;
pub const LIVE_FLAG_LIVEDATA_STREAMING: u8 = 1 << 3;
pub const LIVE_FLAG_WRP_PROTECTED: u8 = 1 << 4;

// ---- DtcEntry (20 B) ----

/// A single DTC table entry. `CMD_DTC_READ` returns
/// `[count_le16, entry_0, entry_1, …]`; each entry is 20 bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DtcEntry {
    pub code: u16,
    pub severity: u8,
    pub occurrence_count: u8,
    pub first_seen_uptime_seconds: u32,
    pub last_seen_uptime_seconds: u32,
    pub context_data: u32,
    pub reserved: u32,
}

impl DtcEntry {
    pub const SIZE: usize = 20;

    pub fn parse(bytes: &[u8]) -> Result<Self, ParseError> {
        ensure_len(bytes, Self::SIZE)?;
        Ok(Self {
            code: read_u16_le(bytes, 0),
            severity: bytes[2],
            occurrence_count: bytes[3],
            first_seen_uptime_seconds: read_u32_le(bytes, 4),
            last_seen_uptime_seconds: read_u32_le(bytes, 8),
            context_data: read_u32_le(bytes, 12),
            reserved: read_u32_le(bytes, 16),
        })
    }

    pub fn severity(&self) -> DtcSeverity {
        DtcSeverity::from_byte(self.severity)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DtcSeverity {
    Info,
    Warn,
    Error,
    Fatal,
    Unknown(u8),
}

impl DtcSeverity {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0 => Self::Info,
            1 => Self::Warn,
            2 => Self::Error,
            3 => Self::Fatal,
            other => Self::Unknown(other),
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Info => "INFO",
            Self::Warn => "WARN",
            Self::Error => "ERROR",
            Self::Fatal => "FATAL",
            Self::Unknown(_) => "UNKNOWN",
        }
    }
}

// ---- ObStatus (16 B) ----

/// Option-byte snapshot returned by `CMD_OB_READ`. `wrp_sector_mask`
/// uses the **HAL convention**: bit N set = sector N WRP-protected.
/// The bootloader's `bl_obyte_read` already inverts the raw register
/// bit sense, so host tooling doesn't need to care.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObStatus {
    pub wrp_sector_mask: u32,
    pub user_config: u32,
    pub rdp_level: u8,
    pub bor_level: u8,
    pub reserved: [u8; 2],
    pub reserved_ext: u32,
}

impl ObStatus {
    pub const SIZE: usize = 16;

    pub fn parse(bytes: &[u8]) -> Result<Self, ParseError> {
        ensure_len(bytes, Self::SIZE)?;
        Ok(Self {
            wrp_sector_mask: read_u32_le(bytes, 0),
            user_config: read_u32_le(bytes, 4),
            rdp_level: bytes[8],
            bor_level: bytes[9],
            reserved: [bytes[10], bytes[11]],
            reserved_ext: read_u32_le(bytes, 12),
        })
    }

    /// `true` if sector `sector` (0..=7) is WRP-protected. Out-of-range
    /// sector numbers return `false` rather than panic.
    pub fn is_sector_protected(&self, sector: u8) -> bool {
        if sector > 7 {
            return false;
        }
        (self.wrp_sector_mask & (1u32 << sector)) != 0
    }
}

/// `BL_OB_APPLY_TOKEN` — the ASCII "WRP\0" LE confirmation token that
/// prefixes every `CMD_OB_APPLY_WRP` request. Host callers should not
/// surface this byte-magic to end users; command builders fill it in
/// automatically.
pub const OB_APPLY_TOKEN: u32 = 0x00505257;

/// `BL_NVM_FORMAT_TOKEN` — the ASCII "FMT\0" LE confirmation token
/// that prefixes every `CMD_NVM_FORMAT` request. Bootloader 0.2+.
/// Same pattern as [`OB_APPLY_TOKEN`] — guards a destructive
/// operation from accidental dispatch by demanding a magic value
/// inline.
pub const NVM_FORMAT_TOKEN: u32 = 0x0054_4D46;

#[cfg(test)]
mod tests {
    use super::*;

    fn le32(n: u32) -> [u8; 4] {
        n.to_le_bytes()
    }
    fn le16(n: u16) -> [u8; 2] {
        n.to_le_bytes()
    }
    fn le64(n: u64) -> [u8; 8] {
        n.to_le_bytes()
    }

    // ---- FirmwareInfo ----

    fn sample_fw_info_bytes() -> Vec<u8> {
        let mut buf = Vec::with_capacity(64);
        buf.extend_from_slice(&le32(FirmwareInfo::MAGIC)); // magic
        buf.extend_from_slice(&le32(0x0001_0000)); // record_version = 1.0
        buf.extend_from_slice(&le32(1)); // major
        buf.extend_from_slice(&le32(4)); // minor
        buf.extend_from_slice(&le32(2)); // patch
        buf.extend_from_slice(&le32(0x0000_0450)); // mcu_id (STM32H7x3)
        buf.extend_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22]); // git_hash
        buf.extend_from_slice(&le64(1_734_567_890)); // build_timestamp
                                                     // product_name: "IFS08-CE-ECU" + NUL pad to 16
        let mut name = [0u8; 16];
        name[..12].copy_from_slice(b"IFS08-CE-ECU");
        buf.extend_from_slice(&name);
        buf.extend_from_slice(&le32(0)); // reserved[0]
        buf.extend_from_slice(&le32(0)); // reserved[1]
        assert_eq!(buf.len(), 64);
        buf
    }

    #[test]
    fn firmware_info_parses() {
        let bytes = sample_fw_info_bytes();
        let fw = FirmwareInfo::parse(&bytes).unwrap();
        assert_eq!(fw.magic, FirmwareInfo::MAGIC);
        assert_eq!(fw.record_version, 0x0001_0000);
        assert_eq!(fw.version(), (1, 4, 2));
        assert_eq!(fw.mcu_id, 0x0000_0450);
        assert_eq!(
            fw.git_hash,
            [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22]
        );
        assert_eq!(fw.build_timestamp, 1_734_567_890);
        assert_eq!(fw.product_name(), "IFS08-CE-ECU");
    }

    #[test]
    fn firmware_info_rejects_bad_magic() {
        let mut bytes = sample_fw_info_bytes();
        bytes[0] = 0xDE;
        let err = FirmwareInfo::parse(&bytes).unwrap_err();
        assert!(matches!(err, ParseError::MagicMismatch { .. }));
    }

    #[test]
    fn firmware_info_rejects_future_major_version() {
        let mut bytes = sample_fw_info_bytes();
        // Bump major to 2 (low 16 bits = minor = 0, high 16 = major = 2)
        bytes[4..8].copy_from_slice(&le32(0x0002_0000));
        let err = FirmwareInfo::parse(&bytes).unwrap_err();
        assert!(matches!(err, ParseError::UnsupportedRecordVersion { .. }));
    }

    #[test]
    fn firmware_info_accepts_higher_minor_version() {
        let mut bytes = sample_fw_info_bytes();
        // Minor = 5, major still 1.
        bytes[4..8].copy_from_slice(&le32(0x0001_0005));
        let fw = FirmwareInfo::parse(&bytes).unwrap();
        assert_eq!(fw.record_version, 0x0001_0005);
    }

    #[test]
    fn firmware_info_rejects_short_buffer() {
        let short = vec![0u8; FirmwareInfo::SIZE - 1];
        let err = FirmwareInfo::parse(&short).unwrap_err();
        assert!(matches!(
            err,
            ParseError::RecordTooShort { got, need } if got == FirmwareInfo::SIZE - 1 && need == FirmwareInfo::SIZE
        ));
    }

    // ---- HealthRecord ----

    #[test]
    fn health_record_parses_flags_and_reset_cause() {
        let mut bytes = Vec::with_capacity(32);
        bytes.extend_from_slice(&le32(42)); // uptime_seconds
        bytes.extend_from_slice(&le32(ResetCause::Wwdg as u32)); // reset_cause
        let flags =
            HEALTH_FLAG_SESSION_ACTIVE | HEALTH_FLAG_VALID_APP_PRESENT | HEALTH_FLAG_WRP_PROTECTED;
        bytes.extend_from_slice(&le32(flags)); // flags
        bytes.extend_from_slice(&le32(0)); // flash_write_count
        bytes.extend_from_slice(&le32(3)); // dtc_count
        bytes.extend_from_slice(&le32(0x0010)); // last_dtc_code
        bytes.extend_from_slice(&[0u8; 8]); // reserved
        assert_eq!(bytes.len(), 32);
        let h = HealthRecord::parse(&bytes).unwrap();
        assert_eq!(h.uptime_seconds, 42);
        assert_eq!(h.reset_cause(), Some(ResetCause::Wwdg));
        assert!(h.session_active());
        assert!(h.valid_app_present());
        assert!(h.wrp_protected());
        assert_eq!(h.dtc_count, 3);
        assert_eq!(h.last_dtc_code, 0x0010);
    }

    // ---- LiveDataSnapshot ----

    #[test]
    fn live_snapshot_parses() {
        let mut bytes = Vec::with_capacity(32);
        bytes.extend_from_slice(&le32(0x1234_5678)); // uptime_ms
        bytes.extend_from_slice(&le16(10)); // frames_rx
        bytes.extend_from_slice(&le16(20)); // frames_tx
        bytes.extend_from_slice(&le16(1)); // nacks_sent
        bytes.extend_from_slice(&le16(2)); // dtc_count
        bytes.extend_from_slice(&le16(0x00F1)); // last_dtc_code
        bytes.push(LIVE_FLAG_SESSION_ACTIVE | LIVE_FLAG_WRP_PROTECTED); // flags
        bytes.push(0x11); // last_opcode
        bytes.extend_from_slice(&le32(0x0802_0000)); // last_flash_addr
        bytes.extend_from_slice(&le32(256)); // isotp_rx_progress
        bytes.extend_from_slice(&le32(5_000)); // session_age_ms
        bytes.extend_from_slice(&le32(0)); // reserved
        assert_eq!(bytes.len(), 32);

        let snap = LiveDataSnapshot::parse(&bytes).unwrap();
        assert_eq!(snap.uptime_ms, 0x1234_5678);
        assert_eq!(snap.frames_rx, 10);
        assert_eq!(snap.frames_tx, 20);
        assert!(snap.session_active());
        assert!(!snap.log_streaming());
        assert!(snap.wrp_protected());
        assert_eq!(snap.last_opcode, 0x11);
        assert_eq!(snap.last_flash_addr, 0x0802_0000);
    }

    // ---- DtcEntry ----

    #[test]
    fn dtc_entry_parses() {
        let mut bytes = Vec::with_capacity(20);
        bytes.extend_from_slice(&le16(0x0010)); // code = FLASH_HW
        bytes.push(2); // severity = ERROR
        bytes.push(3); // occurrence_count
        bytes.extend_from_slice(&le32(100)); // first_seen
        bytes.extend_from_slice(&le32(250)); // last_seen
        bytes.extend_from_slice(&le32(0x0802_0040)); // context_data
        bytes.extend_from_slice(&le32(0)); // reserved
        assert_eq!(bytes.len(), 20);

        let e = DtcEntry::parse(&bytes).unwrap();
        assert_eq!(e.code, 0x0010);
        assert_eq!(e.severity(), DtcSeverity::Error);
        assert_eq!(e.occurrence_count, 3);
        assert_eq!(e.first_seen_uptime_seconds, 100);
        assert_eq!(e.last_seen_uptime_seconds, 250);
    }

    // ---- ObStatus ----

    #[test]
    fn ob_status_parses_and_queries_sectors() {
        let mut bytes = Vec::with_capacity(16);
        // Sectors 0 and 3 protected.
        bytes.extend_from_slice(&le32(0b0000_1001));
        bytes.extend_from_slice(&le32(0xFFFF_F2AA));
        bytes.push(0xAA); // RDP level 1 ("AA" -> level 0 on H7, value chosen for the test)
        bytes.push(0x07); // BOR level 3
        bytes.extend_from_slice(&[0, 0]);
        bytes.extend_from_slice(&le32(0));
        assert_eq!(bytes.len(), 16);

        let ob = ObStatus::parse(&bytes).unwrap();
        assert!(ob.is_sector_protected(0));
        assert!(!ob.is_sector_protected(1));
        assert!(!ob.is_sector_protected(2));
        assert!(ob.is_sector_protected(3));
        assert!(!ob.is_sector_protected(7));
        assert!(!ob.is_sector_protected(8)); // out-of-range returns false
        assert_eq!(ob.rdp_level, 0xAA);
        assert_eq!(ob.bor_level, 0x07);
    }

    #[test]
    fn ob_apply_token_bytes_are_wrp_nul_le() {
        // "WRP\0" little-endian = W(0x57), R(0x52), P(0x50), 0x00
        assert_eq!(OB_APPLY_TOKEN.to_le_bytes(), [0x57, 0x52, 0x50, 0x00]);
    }
}
