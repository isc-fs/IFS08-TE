//! Typed command-payload builders.
//!
//! Every function in this module returns a `Vec<u8>` formatted as
//! `[opcode, args_le…]`, ready to hand to [`super::isotp::IsoTpSegmenter`].
//! No transport I/O happens here — this is pure bit-packing.
//!
//! Keeping the builders standalone (rather than methods on a
//! `CommandBuilder`) is deliberate. Each one is one line of call
//! site code:
//!
//! ```
//! use can_flasher::protocol::commands::cmd_flash_erase;
//! let payload = cmd_flash_erase(0x0802_0000, 0x20000);
//! ```
//!
//! Callers that want typed wrappers can layer their own enum on top.

use super::opcodes::{CommandOpcode, ResetMode};
use super::records::{NVM_FORMAT_TOKEN, OB_APPLY_TOKEN};

/// Protocol version advertised by this build in `CMD_CONNECT`.
/// Bootloader only strict-equals against the major byte; minor is
/// advisory. We track the bootloader's `BL_PROTO_VERSION_MAJOR /
/// MINOR` even so, so a `discover --json` dump documents which
/// contract the flasher targets.
pub const PROTOCOL_VERSION_MAJOR: u8 = 0;
/// Protocol version advertised by this build in `CMD_CONNECT`.
/// Bumped 0.1 → 0.2 to track the bootloader's 0.2 release (which
/// added `CMD_NVM_FORMAT` and `BL_NACK_NVM_WRONG_TOKEN`).
pub const PROTOCOL_VERSION_MINOR: u8 = 2;

fn payload_with_opcode(opcode: CommandOpcode, args: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(1 + args.len());
    v.push(opcode.as_byte());
    v.extend_from_slice(args);
    v
}

/// `CMD_CONNECT [major, minor]`.
pub fn cmd_connect(major: u8, minor: u8) -> Vec<u8> {
    payload_with_opcode(CommandOpcode::Connect, &[major, minor])
}

/// `CMD_CONNECT` with the host's own advertised protocol version.
pub fn cmd_connect_self() -> Vec<u8> {
    cmd_connect(PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR)
}

/// `CMD_DISCONNECT` — no args.
pub fn cmd_disconnect() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::Disconnect, &[])
}

/// `CMD_DISCOVER` — no args. Caller must send as `TYPE=DISCOVER`,
/// `dst=0xF`.
pub fn cmd_discover() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::Discover, &[])
}

/// `CMD_GET_FW_INFO` — no args.
pub fn cmd_get_fw_info() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::GetFwInfo, &[])
}

/// `CMD_GET_HEALTH` — no args.
pub fn cmd_get_health() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::GetHealth, &[])
}

/// `CMD_FLASH_ERASE [start_le32, length_le32]`.
pub fn cmd_flash_erase(start: u32, length: u32) -> Vec<u8> {
    let mut args = [0u8; 8];
    args[0..4].copy_from_slice(&start.to_le_bytes());
    args[4..8].copy_from_slice(&length.to_le_bytes());
    payload_with_opcode(CommandOpcode::FlashErase, &args)
}

/// `CMD_FLASH_WRITE [addr_le32, data…]`. `data.len()` must fit in a
/// single ISO-TP message (≤ `MAX_MSG_LEN - 5` bytes of actual data).
pub fn cmd_flash_write(addr: u32, data: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(1 + 4 + data.len());
    v.push(CommandOpcode::FlashWrite.as_byte());
    v.extend_from_slice(&addr.to_le_bytes());
    v.extend_from_slice(data);
    v
}

/// `CMD_FLASH_READ_CRC [addr_le32, length_le32]`.
pub fn cmd_flash_read_crc(addr: u32, length: u32) -> Vec<u8> {
    let mut args = [0u8; 8];
    args[0..4].copy_from_slice(&addr.to_le_bytes());
    args[4..8].copy_from_slice(&length.to_le_bytes());
    payload_with_opcode(CommandOpcode::FlashReadCrc, &args)
}

/// `CMD_FLASH_VERIFY [expected_crc_le32, expected_size_le32,
/// expected_version_le32]`. Triggers the bootloader to re-CRC the
/// installed image and commit the metadata FLASHWORD on match.
pub fn cmd_flash_verify(expected_crc: u32, expected_size: u32, expected_version: u32) -> Vec<u8> {
    let mut args = [0u8; 12];
    args[0..4].copy_from_slice(&expected_crc.to_le_bytes());
    args[4..8].copy_from_slice(&expected_size.to_le_bytes());
    args[8..12].copy_from_slice(&expected_version.to_le_bytes());
    payload_with_opcode(CommandOpcode::FlashVerify, &args)
}

/// `CMD_LOG_STREAM_START [min_severity]`.
pub fn cmd_log_stream_start(min_severity: u8) -> Vec<u8> {
    payload_with_opcode(CommandOpcode::LogStreamStart, &[min_severity])
}

/// `CMD_LOG_STREAM_STOP` — no args.
pub fn cmd_log_stream_stop() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::LogStreamStop, &[])
}

/// `CMD_LIVE_DATA_START [rate_hz]`. The bootloader clamps
/// `rate_hz` to `1..=50`; out-of-range values earn NACK(UNSUPPORTED).
pub fn cmd_live_data_start(rate_hz: u8) -> Vec<u8> {
    payload_with_opcode(CommandOpcode::LiveDataStart, &[rate_hz])
}

/// `CMD_LIVE_DATA_STOP` — no args.
pub fn cmd_live_data_stop() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::LiveDataStop, &[])
}

/// `CMD_DTC_READ` — no args.
pub fn cmd_dtc_read() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::DtcRead, &[])
}

/// `CMD_DTC_CLEAR` — no args.
pub fn cmd_dtc_clear() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::DtcClear, &[])
}

/// `CMD_OB_READ` — no args.
pub fn cmd_ob_read() -> Vec<u8> {
    payload_with_opcode(CommandOpcode::ObRead, &[])
}

/// `CMD_OB_APPLY_WRP [token_le32, sector_bitmap_le32?]`.
///
/// The 4-byte confirmation token is filled in automatically from
/// [`OB_APPLY_TOKEN`]; callers don't need to — and shouldn't — know
/// the magic value. When `sector_bitmap` is `None` the field is
/// omitted, which lets the bootloader's default (`0x01`, protect
/// sector 0) kick in. When `Some`, the 4 bytes are appended.
pub fn cmd_ob_apply_wrp(sector_bitmap: Option<u32>) -> Vec<u8> {
    let mut v = Vec::with_capacity(1 + 4 + 4);
    v.push(CommandOpcode::ObApplyWrp.as_byte());
    v.extend_from_slice(&OB_APPLY_TOKEN.to_le_bytes());
    if let Some(mask) = sector_bitmap {
        v.extend_from_slice(&mask.to_le_bytes());
    }
    v
}

/// `CMD_RESET [mode]`.
pub fn cmd_reset(mode: ResetMode) -> Vec<u8> {
    payload_with_opcode(CommandOpcode::Reset, &[mode.as_byte()])
}

/// `CMD_JUMP [addr_le32]`.
pub fn cmd_jump(addr: u32) -> Vec<u8> {
    payload_with_opcode(CommandOpcode::Jump, &addr.to_le_bytes())
}

/// `CMD_NVM_READ [key_le16]`.
pub fn cmd_nvm_read(key: u16) -> Vec<u8> {
    payload_with_opcode(CommandOpcode::NvmRead, &key.to_le_bytes())
}

/// `CMD_NVM_WRITE [key_le16, value…]`. Passing `value.len() == 0`
/// is a tombstone. Caller enforces the bootloader's
/// `BL_NVM_MAX_VALUE_LEN` cap.
pub fn cmd_nvm_write(key: u16, value: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(1 + 2 + value.len());
    v.push(CommandOpcode::NvmWrite.as_byte());
    v.extend_from_slice(&key.to_le_bytes());
    v.extend_from_slice(value);
    v
}

/// `CMD_NVM_FORMAT [token_le32]`. Bootloader 0.2+ only.
///
/// Erases the NVM sector unconditionally — every key + the
/// metadata FLASHWORD — and resets the bootloader's internal NVM
/// pointers. The 4-byte confirmation token is filled in
/// automatically from [`NVM_FORMAT_TOKEN`]; callers don't need to
/// — and shouldn't — know the magic value. Wrong / missing token
/// NACKs with [`NackCode::NvmWrongToken`].
///
/// [`NackCode::NvmWrongToken`]: super::opcodes::NackCode::NvmWrongToken
pub fn cmd_nvm_format() -> Vec<u8> {
    let mut v = Vec::with_capacity(1 + 4);
    v.push(CommandOpcode::NvmFormat.as_byte());
    v.extend_from_slice(&NVM_FORMAT_TOKEN.to_le_bytes());
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn connect_payload_shape() {
        let p = cmd_connect(0, 1);
        assert_eq!(p, vec![0x01, 0, 1]);
    }

    #[test]
    fn connect_self_uses_advertised_version() {
        let p = cmd_connect_self();
        assert_eq!(
            p,
            vec![0x01, PROTOCOL_VERSION_MAJOR, PROTOCOL_VERSION_MINOR]
        );
    }

    #[test]
    fn flash_erase_le_encoding() {
        let p = cmd_flash_erase(0x0802_0000, 0x00020000);
        assert_eq!(p[0], 0x10);
        // start
        assert_eq!(&p[1..5], &[0x00, 0x00, 0x02, 0x08]);
        // length
        assert_eq!(&p[5..9], &[0x00, 0x00, 0x02, 0x00]);
        assert_eq!(p.len(), 9);
    }

    #[test]
    fn flash_verify_carries_12_byte_args() {
        let p = cmd_flash_verify(0xDEAD_BEEF, 0x18000, 0x0001_0402);
        assert_eq!(p[0], 0x13);
        assert_eq!(&p[1..5], &0xDEAD_BEEFu32.to_le_bytes());
        assert_eq!(&p[5..9], &0x0001_8000u32.to_le_bytes());
        assert_eq!(&p[9..13], &0x0001_0402u32.to_le_bytes());
        assert_eq!(p.len(), 13);
    }

    #[test]
    fn ob_apply_wrp_prepends_token() {
        let p = cmd_ob_apply_wrp(None);
        assert_eq!(p[0], 0x51);
        // Token bytes: "WRP\0" LE = 0x57, 0x52, 0x50, 0x00.
        assert_eq!(&p[1..5], &[0x57, 0x52, 0x50, 0x00]);
        assert_eq!(p.len(), 5);
    }

    #[test]
    fn ob_apply_wrp_with_mask_appends_4_bytes() {
        let p = cmd_ob_apply_wrp(Some(0x03));
        assert_eq!(&p[1..5], &[0x57, 0x52, 0x50, 0x00]);
        assert_eq!(&p[5..9], &0x0000_0003u32.to_le_bytes());
        assert_eq!(p.len(), 9);
    }

    #[test]
    fn reset_payload_is_single_mode_byte() {
        let p = cmd_reset(ResetMode::Bootloader);
        assert_eq!(p, vec![0x60, 2]);
    }

    #[test]
    fn jump_payload_is_4_byte_addr() {
        let p = cmd_jump(0x0802_0000);
        assert_eq!(p[0], 0x61);
        assert_eq!(&p[1..5], &0x0802_0000u32.to_le_bytes());
        assert_eq!(p.len(), 5);
    }

    #[test]
    fn nvm_read_is_key_only() {
        let p = cmd_nvm_read(0x1234);
        assert_eq!(p, vec![0x80, 0x34, 0x12]);
    }

    #[test]
    fn nvm_write_tombstone_has_zero_value_bytes() {
        let p = cmd_nvm_write(0x0001, &[]);
        assert_eq!(p, vec![0x81, 0x01, 0x00]);
    }

    #[test]
    fn nvm_write_carries_value() {
        let p = cmd_nvm_write(0x1000, b"hello");
        assert_eq!(p[0], 0x81);
        assert_eq!(&p[1..3], &[0x00, 0x10]);
        assert_eq!(&p[3..8], b"hello");
    }

    #[test]
    fn log_and_livedata_start_single_arg() {
        assert_eq!(cmd_log_stream_start(2), vec![0x30, 2]);
        assert_eq!(cmd_live_data_start(10), vec![0x32, 10]);
    }

    #[test]
    fn nvm_format_carries_token_only() {
        let p = cmd_nvm_format();
        assert_eq!(p[0], 0x82);
        // 'F','M','T',0x00 little-endian.
        assert_eq!(&p[1..5], &[0x46, 0x4D, 0x54, 0x00]);
        assert_eq!(p.len(), 5);
    }
}
