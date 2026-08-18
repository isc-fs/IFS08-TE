//! 11-bit standard CAN ID encoding used by the bootloader protocol.
//!
//! ## Wire layout (Proposal A — fix/12)
//!
//! ```text
//!   bits 10..5 = 0           (reserved; keeps every valid ID ≤ 0x01F)
//!   bit  4     = direction   0 = host→node, 1 = node→host
//!   bits 3..0  = other-end node ID
//!                host→node: dst (0x0..0xE unicast, 0xF broadcast)
//!                node→host: src (0x1..0xE; 0x0 and 0xF reserved)
//! ```
//!
//! Every BL and app on the bus uses the same layout; the 16 possible
//! node IDs are assigned per-board via provisioning. The top six bits
//! are always zero, which guarantees every valid ID sits in
//! `0x000..=0x01F` — well below 0x30 for CAN arbitration priority over
//! any non-BL traffic sharing the bus, and leaves `0x020..=0x7FF`
//! entirely free for future protocol extensions.
//!
//! ## Concrete ID ranges
//!
//! | Range           | Purpose                                              |
//! |-----------------|------------------------------------------------------|
//! | `0x000..=0x00E` | host → node CMD + host-side ISO-TP CFs               |
//! | `0x00F`         | host → broadcast DISCOVER (and app-ctrl broadcasts)  |
//! | `0x010..=0x01E` | node → host ACK/NACK/NOTIFY/DISCOVER-reply/CF/FC     |
//! | `0x01F`         | reserved                                             |
//!
//! ## Message type byte
//!
//! Today's 3-bit type field in the ID moved into payload byte 1 of
//! every SF/FF (CFs/FCs inherit from their parent FF, so they don't
//! carry the byte). [`MessageType`] in this module holds the wire
//! values — they're identical to the old type nibble numbers, minus
//! `Data` (which doesn't exist as a type in the new layout; CFs ride
//! on their parent message's ID) and with `Discover` split into
//! [`MessageType::DiscoverRequest`] + [`MessageType::DiscoverReply`]
//! (so host-side and device-side flows can be distinguished in parse
//! paths without inferring from direction). A new
//! [`MessageType::AppCtrl`] is reserved for application-level
//! traffic: the BL silently drops these, app firmware handles them.

use super::ParseError;

/// Message type, carried as the first payload byte after the ISO-TP
/// PCI in every SF/FF. The BL and host both read this to route the
/// message; CFs and FCs don't carry it (ISO-TP PCI is sufficient to
/// identify them as continuation traffic).
///
/// Wire values match the numeric constants in the bootloader's
/// `bl_proto.h`. Keep in sync.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MessageType {
    /// Host → device command frame.
    Cmd = 0x00,
    /// Device → host positive acknowledgement.
    Ack = 0x01,
    /// Device → host negative acknowledgement (carries a NACK code).
    Nack = 0x02,
    /// Device → host unsolicited event (heartbeat, DTC, log, live
    /// data).
    Notify = 0x03,
    /// Host → broadcast discovery ping. Only ever sent with
    /// dst=`BROADCAST_NODE_ID`. Split out from the old combined
    /// `Discover` type so reply and request are unambiguous in parse
    /// paths.
    DiscoverRequest = 0x04,
    /// Device → host discovery reply.
    DiscoverReply = 0x05,
    /// Application-level command. Host → node. The BL silently drops
    /// these; the application firmware running after BL-jump handles
    /// them. Used by [`cli::send_raw`](crate::cli::send_raw) for the
    /// "reboot back to BL" handshake and by any app-specific
    /// conventions layered on top.
    AppCtrl = 0x06,
}

impl MessageType {
    /// Parse a raw msg_type byte. Returns `Err` for unknown values
    /// (`0x07..=0xFF`) so parse paths stay strict.
    pub fn from_byte(byte: u8) -> Result<Self, ParseError> {
        match byte {
            0x00 => Ok(Self::Cmd),
            0x01 => Ok(Self::Ack),
            0x02 => Ok(Self::Nack),
            0x03 => Ok(Self::Notify),
            0x04 => Ok(Self::DiscoverRequest),
            0x05 => Ok(Self::DiscoverReply),
            0x06 => Ok(Self::AppCtrl),
            other => Err(ParseError::UnknownMessageType(other)),
        }
    }

    /// Wire byte value, suitable for placing into payload[1] of
    /// SF/FF frames.
    pub fn as_byte(self) -> u8 {
        self as u8
    }
}

/// Reserved node ID for the host. Always 0x0.
///
/// The new layout has no `src` field in the ID (direction is the only
/// source indicator), but the host-is-0 convention carries through:
/// the BL's heartbeat/DTC/log notifications still stamp 0x0 into
/// payload fields that historically called out the destination.
pub const HOST_NODE_ID: u8 = 0x0;

/// Reserved node ID for broadcast addressing. Always 0xF. Only valid
/// in the host→node direction.
pub const BROADCAST_NODE_ID: u8 = 0xF;

// ---- Bit-field masks ----

/// Bit 4 of the 11-bit ID. Set = node→host; clear = host→node.
const DIRECTION_BIT: u16 = 0x10;

/// Low-nibble mask for the node-ID field.
const NODE_MASK: u16 = 0x0F;

/// Combined mask matching the five low bits we actually use. Every
/// valid ID satisfies `id & !VALID_ID_MASK == 0`.
pub const VALID_ID_MASK: u16 = 0x1F;

/// Which end of the bus produced this frame. Along with
/// [`FrameId::node`] it fully describes the addressing of the frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameDirection {
    /// Host sent this frame; the node identified by `node` is the
    /// destination. `node = BROADCAST_NODE_ID` = broadcast.
    HostToNode,
    /// Node sent this frame; the node identified by `node` is the
    /// source. `node = HOST_NODE_ID` and `node = BROADCAST_NODE_ID`
    /// are both reserved in this direction — a bootloader never
    /// spoofs the host address nor the broadcast address.
    NodeToHost,
}

/// Decoded form of the 11-bit frame ID.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameId {
    /// Who sent the frame.
    pub direction: FrameDirection,
    /// The "other end" node, interpreted per `direction`:
    /// - `HostToNode`: `node` is the destination (or `0xF` broadcast).
    /// - `NodeToHost`: `node` is the source (1..=0xE).
    pub node: u8,
}

impl FrameId {
    /// Build a frame ID from its components. Validates:
    /// - `node` fits in 4 bits.
    /// - For `NodeToHost`, `node` is 0x1..=0xE (0x0 = host, 0xF =
    ///   broadcast are both reserved on this direction).
    pub fn new(direction: FrameDirection, node: u8) -> Result<Self, ParseError> {
        if u16::from(node) > NODE_MASK {
            return Err(ParseError::NodeIdOutOfRange(node));
        }
        if matches!(direction, FrameDirection::NodeToHost)
            && (node == HOST_NODE_ID || node == BROADCAST_NODE_ID)
        {
            return Err(ParseError::NodeIdOutOfRange(node));
        }
        Ok(Self { direction, node })
    }

    /// Convenience: host-originated frame addressed to `dst`.
    pub fn from_host(dst: u8) -> Result<Self, ParseError> {
        Self::new(FrameDirection::HostToNode, dst)
    }

    /// Convenience: node-originated frame with the given `src`.
    pub fn from_node(src: u8) -> Result<Self, ParseError> {
        Self::new(FrameDirection::NodeToHost, src)
    }

    /// Encode into the 11-bit standard identifier.
    pub fn encode(self) -> u16 {
        let dir_bit = match self.direction {
            FrameDirection::HostToNode => 0,
            FrameDirection::NodeToHost => DIRECTION_BIT,
        };
        dir_bit | (u16::from(self.node) & NODE_MASK)
    }

    /// Parse an 11-bit ID into its components. Rejects any ID with
    /// bits set outside the low 5, and rejects reserved node→host
    /// node IDs (0x0 = would mean host-spoofed-as-node, 0xF = would
    /// mean broadcast-from-a-node which is nonsense).
    pub fn decode(raw: u16) -> Result<Self, ParseError> {
        if raw & !VALID_ID_MASK != 0 {
            return Err(ParseError::IdOutOfRange(raw));
        }
        let node = (raw & NODE_MASK) as u8;
        let direction = if raw & DIRECTION_BIT != 0 {
            FrameDirection::NodeToHost
        } else {
            FrameDirection::HostToNode
        };
        // Reserved-node check applies only to NodeToHost.
        if matches!(direction, FrameDirection::NodeToHost)
            && (node == HOST_NODE_ID || node == BROADCAST_NODE_ID)
        {
            return Err(ParseError::IdOutOfRange(raw));
        }
        Ok(Self { direction, node })
    }

    /// True if this frame is addressed to the given node. Only
    /// meaningful for `HostToNode` frames (nodes on the bus can see
    /// every frame, but only act on ones dst'd to them or the
    /// broadcast address). A `NodeToHost` frame is always addressed
    /// to the host implicitly — callers rarely need to ask this.
    pub fn addressed_to(&self, node_id: u8) -> bool {
        match self.direction {
            FrameDirection::HostToNode => self.node == node_id || self.node == BROADCAST_NODE_ID,
            // Node-to-host frames are bound for the host; no other
            // node is the intended recipient.
            FrameDirection::NodeToHost => node_id == HOST_NODE_ID,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_host_to_node_unicast() {
        // host → node 0x3 → 0x003
        assert_eq!(FrameId::from_host(0x3).unwrap().encode(), 0x003);
        // host → node 0xE → 0x00E
        assert_eq!(FrameId::from_host(0xE).unwrap().encode(), 0x00E);
    }

    #[test]
    fn encode_host_to_node_broadcast() {
        // host → broadcast → 0x00F
        assert_eq!(
            FrameId::from_host(BROADCAST_NODE_ID).unwrap().encode(),
            0x00F
        );
    }

    #[test]
    fn encode_node_to_host() {
        // node 0x1 → host → 0x011
        assert_eq!(FrameId::from_node(0x1).unwrap().encode(), 0x011);
        // node 0xE → host → 0x01E
        assert_eq!(FrameId::from_node(0xE).unwrap().encode(), 0x01E);
    }

    #[test]
    fn decode_is_inverse_of_encode() {
        // Every valid host→node frame.
        for dst in 0x0..=0xF {
            let id = FrameId::from_host(dst).unwrap();
            assert_eq!(FrameId::decode(id.encode()).unwrap(), id);
        }
        // Every valid node→host frame.
        for src in 0x1..=0xE {
            let id = FrameId::from_node(src).unwrap();
            assert_eq!(FrameId::decode(id.encode()).unwrap(), id);
        }
    }

    #[test]
    fn decode_rejects_id_with_bits_above_bit_4() {
        assert!(matches!(
            FrameId::decode(0x020),
            Err(ParseError::IdOutOfRange(0x020))
        ));
        assert!(matches!(
            FrameId::decode(0x100),
            Err(ParseError::IdOutOfRange(0x100))
        ));
        assert!(matches!(
            FrameId::decode(0x7FF),
            Err(ParseError::IdOutOfRange(0x7FF))
        ));
    }

    #[test]
    fn decode_rejects_reserved_node_to_host_nodes() {
        // 0x010 = node→host with src=0x0 (host spoofed) — rejected.
        assert!(matches!(
            FrameId::decode(0x010),
            Err(ParseError::IdOutOfRange(0x010))
        ));
        // 0x01F = node→host with src=0xF (broadcast-from-node) —
        // rejected.
        assert!(matches!(
            FrameId::decode(0x01F),
            Err(ParseError::IdOutOfRange(0x01F))
        ));
    }

    #[test]
    fn new_rejects_oversize_node_ids() {
        assert!(matches!(
            FrameId::new(FrameDirection::HostToNode, 0x10),
            Err(ParseError::NodeIdOutOfRange(0x10))
        ));
        assert!(matches!(
            FrameId::new(FrameDirection::HostToNode, 0xFF),
            Err(ParseError::NodeIdOutOfRange(0xFF))
        ));
    }

    #[test]
    fn new_rejects_reserved_node_to_host_nodes() {
        // Host can't claim to be a node.
        assert!(FrameId::from_node(HOST_NODE_ID).is_err());
        // Broadcast is a destination, not a source.
        assert!(FrameId::from_node(BROADCAST_NODE_ID).is_err());
    }

    #[test]
    fn addressed_to_catches_broadcast_and_unicast() {
        let my_id = 0x3;
        let unicast = FrameId::from_host(my_id).unwrap();
        let broadcast = FrameId::from_host(BROADCAST_NODE_ID).unwrap();
        let other = FrameId::from_host(0x4).unwrap();
        assert!(unicast.addressed_to(my_id));
        assert!(broadcast.addressed_to(my_id));
        assert!(!other.addressed_to(my_id));
    }

    #[test]
    fn addressed_to_for_node_to_host_targets_the_host() {
        let reply = FrameId::from_node(0x3).unwrap();
        assert!(reply.addressed_to(HOST_NODE_ID));
        // Other nodes shouldn't treat a node-to-host reply as for them.
        assert!(!reply.addressed_to(0x5));
    }

    #[test]
    fn message_type_round_trips() {
        for &mt in &[
            MessageType::Cmd,
            MessageType::Ack,
            MessageType::Nack,
            MessageType::Notify,
            MessageType::DiscoverRequest,
            MessageType::DiscoverReply,
            MessageType::AppCtrl,
        ] {
            assert_eq!(MessageType::from_byte(mt.as_byte()).unwrap(), mt);
        }
    }

    #[test]
    fn message_type_rejects_unknown_values() {
        // Values 0x07..=0xFF are reserved.
        assert!(matches!(
            MessageType::from_byte(0x07),
            Err(ParseError::UnknownMessageType(0x07))
        ));
        assert!(matches!(
            MessageType::from_byte(0xFF),
            Err(ParseError::UnknownMessageType(0xFF))
        ));
    }
}
