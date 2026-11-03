//! Reassembly of camera frames from isochronous packets, with no I/O.
//!
//! Each packet starts with a 12-byte header: `"RB"`, a pad byte, a flag, an unknown byte, a
//! sequence number, two unknown bytes and a little-endian `u32` timestamp. The flag's high nibble
//! names the stream (0x7_ depth, 0x8_ video) and its low nibble the packet's place in the frame:
//! 1 first, 2 middle, 5 last.

pub const PACKET_HEADER_LEN: usize = 12;

const START: u8 = 1;
const MIDDLE: u8 = 2;
const END: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamKind {
    Depth,
    Video,
}

impl StreamKind {
    fn flag(self) -> u8 {
        match self {
            Self::Depth => 0x70,
            Self::Video => 0x80,
        }
    }

    /// Frame bytes carried by every packet but the last.
    pub fn payload_len(self) -> usize {
        match self {
            Self::Depth => 1760 - PACKET_HEADER_LEN,
            Self::Video => 1920 - PACKET_HEADER_LEN,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacketHeader {
    pub flag: u8,
    pub sequence: u8,
    pub timestamp: u32,
}

/// Splits a packet into its header and payload. Returns `None` if it is not a stream packet.
pub fn parse_packet(packet: &[u8]) -> Option<(PacketHeader, &[u8])> {
    if packet.len() < PACKET_HEADER_LEN || packet[..2] != *b"RB" {
        return None;
    }
    let header = PacketHeader {
        flag: packet[3],
        sequence: packet[5],
        timestamp: u32::from_le_bytes(packet[8..12].try_into().unwrap()),
    };
    Some((header, &packet[PACKET_HEADER_LEN..]))
}

/// A complete frame in the camera's packed wire format.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawFrame {
    pub data: Vec<u8>,
    /// Camera timestamp, from the frame's last packet (the first one carries 0).
    pub timestamp: u32,
}

/// Collects packets into frames, dropping any frame that loses a packet or gets a short one.
pub struct FrameAssembler {
    kind: StreamKind,
    frame: Vec<u8>,
    packets_per_frame: usize,
    /// Frames end at their last packet, wherever that falls, rather than at a fixed size.
    variable: bool,
    synced: bool,
    /// Index within the frame of the next expected packet.
    packet: usize,
    sequence: u8,
    dropped: u64,
}

impl FrameAssembler {
    /// Assembles frames of exactly `frame_len` bytes.
    pub fn new(kind: StreamKind, frame_len: usize) -> Self {
        Self {
            kind,
            frame: vec![0; frame_len],
            packets_per_frame: frame_len.div_ceil(kind.payload_len()),
            variable: false,
            synced: false,
            packet: 0,
            sequence: 0,
            dropped: 0,
        }
    }

    /// Assembles frames of up to `max_len` bytes that end wherever their last packet comes,
    /// for modes whose size differs between Kinect models.
    pub fn variable(kind: StreamKind, max_len: usize) -> Self {
        Self { variable: true, ..Self::new(kind, max_len) }
    }

    /// Frames abandoned because packets were lost or out of place.
    pub fn dropped_frames(&self) -> u64 {
        self.dropped
    }

    fn lose_frame(&mut self) {
        if self.synced && self.packet > 0 {
            self.dropped += 1;
        }
        self.synced = false;
    }

    /// Adds one isochronous packet. Returns a frame when this packet completes one.
    pub fn push(&mut self, packet: &[u8]) -> Option<RawFrame> {
        let (header, payload) = parse_packet(packet)?;
        let flag = self.kind.flag();
        if header.flag & 0xf0 != flag {
            return None;
        }
        if self.synced && header.sequence != self.sequence {
            self.lose_frame();
        }
        if !self.synced {
            if header.flag != flag | START {
                return None;
            }
            self.synced = true;
            self.packet = 0;
            self.sequence = header.sequence;
        }

        let last = if self.variable {
            self.packet > 0 && header.flag == flag | END
        } else {
            self.packet + 1 == self.packets_per_frame
        };
        let expected = flag
            | match (self.packet, last) {
                (0, _) => START,
                (_, true) => END,
                _ => MIDDLE,
            };
        let start = self.packet * self.kind.payload_len();
        let room = self.frame.len().saturating_sub(start).min(self.kind.payload_len());
        // Only the last packet may be short, and only a variable-length frame's.
        let fits = payload.len() == room || (self.variable && last && payload.len() <= room);
        if header.flag != expected || !fits || room == 0 {
            self.lose_frame();
            // A misplaced first packet still starts the next frame.
            return if header.flag == flag | START { self.push(packet) } else { None };
        }

        self.frame[start..start + payload.len()].copy_from_slice(payload);
        self.packet += 1;
        self.sequence = header.sequence.wrapping_add(1);
        if !last {
            return None;
        }
        self.packet = 0;
        let len = self.frame.len();
        let mut data = std::mem::replace(&mut self.frame, vec![0; len]);
        data.truncate(start + payload.len());
        Some(RawFrame { data, timestamp: header.timestamp })
    }
}

/// Unpacks samples of `bits` bits stored most significant bit first, as the camera sends its
/// 10- and 11-bit formats. Fills all of `out`.
pub fn unpack_bits(raw: &[u8], bits: u32, out: &mut [u16]) {
    let mask = (1u32 << bits) - 1;
    let mut bytes = raw.iter();
    let mut acc = 0u32;
    let mut have = 0;
    for sample in out {
        while have < bits {
            acc = (acc << 8) | u32::from(bytes.next().copied().unwrap_or(0));
            have += 8;
        }
        have -= bits;
        *sample = ((acc >> have) & mask) as u16;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(flag: u8, sequence: u8, timestamp: u32, payload: &[u8]) -> Vec<u8> {
        let mut p = vec![b'R', b'B', 0, flag, 0, sequence, 0, 0];
        p.extend(timestamp.to_le_bytes());
        p.extend(payload);
        p
    }

    /// A depth frame of 2.5 packets' worth of bytes: three packets, the last one short.
    fn frame_packets(first_sequence: u8, timestamp: u32) -> (Vec<Vec<u8>>, Vec<u8>) {
        let n = StreamKind::Depth.payload_len();
        let data: Vec<u8> = (0..n * 5 / 2).map(|i| i as u8).collect();
        let packets = vec![
            packet(0x71, first_sequence, timestamp, &data[..n]),
            packet(0x72, first_sequence.wrapping_add(1), timestamp + 1, &data[n..2 * n]),
            packet(0x75, first_sequence.wrapping_add(2), timestamp + 2, &data[2 * n..]),
        ];
        (packets, data)
    }

    #[test]
    fn parses_packet_header() {
        let p = packet(0x75, 9, 0x01020304, &[1, 2]);
        let (header, payload) = parse_packet(&p).unwrap();
        assert_eq!(header, PacketHeader { flag: 0x75, sequence: 9, timestamp: 0x01020304 });
        assert_eq!(payload, [1, 2]);
        assert!(parse_packet(&p[..11]).is_none());
        assert!(parse_packet(b"GM\0\0\0\0\0\0\0\0\0\0").is_none());
    }

    #[test]
    fn assembles_consecutive_frames() {
        let (first, data) = frame_packets(254, 100);
        let len = data.len();
        let mut assembler = FrameAssembler::new(StreamKind::Depth, len);
        // Packets from mid-frame and from the video stream are ignored until a frame starts.
        assert!(assembler.push(&packet(0x72, 3, 0, &[0; 4])).is_none());
        assert!(assembler.push(&packet(0x81, 3, 0, &[0; 4])).is_none());

        assert!(assembler.push(&first[0]).is_none());
        assert!(assembler.push(&first[1]).is_none());
        let frame = assembler.push(&first[2]).unwrap();
        assert_eq!(frame, RawFrame { data: data.clone(), timestamp: 102 });

        let (second, _) = frame_packets(1, 200); // sequence continues across the u8 wrap
        let frames: Vec<_> = second.iter().filter_map(|p| assembler.push(p)).collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].timestamp, 202);
        assert_eq!(assembler.dropped_frames(), 0);
    }

    #[test]
    fn drops_frames_with_lost_packets_and_resyncs() {
        let (first, data) = frame_packets(10, 100);
        let mut assembler = FrameAssembler::new(StreamKind::Depth, data.len());
        assert!(assembler.push(&first[0]).is_none());
        assert!(assembler.push(&first[2]).is_none(), "middle packet lost");
        assert_eq!(assembler.dropped_frames(), 1);

        let (second, _) = frame_packets(20, 200);
        let frames: Vec<_> = second.iter().filter_map(|p| assembler.push(p)).collect();
        assert_eq!(frames.len(), 1);

        let (mut third, _) = frame_packets(23, 300);
        third[2].pop();
        assert!(third.iter().all(|p| assembler.push(p).is_none()), "short last packet");
        assert_eq!(assembler.dropped_frames(), 2);
    }

    #[test]
    fn restarts_on_unexpected_first_packet() {
        let (first, data) = frame_packets(10, 100);
        let (second, _) = frame_packets(11, 200);
        let mut assembler = FrameAssembler::new(StreamKind::Depth, data.len());
        assert!(assembler.push(&first[0]).is_none());
        // A new frame starts where packet 2 of the first was expected.
        assert!(assembler.push(&second[0]).is_none());
        assert!(assembler.push(&second[1]).is_none());
        assert_eq!(assembler.push(&second[2]).unwrap().timestamp, 202);
        assert_eq!(assembler.dropped_frames(), 1);
    }

    #[test]
    fn variable_frames_end_at_their_last_packet() {
        let n = StreamKind::Video.payload_len();
        let mut assembler = FrameAssembler::variable(StreamKind::Video, n * 4);
        let frame: Vec<_> =
            [packet(0x81, 0, 0, &vec![1; n]), packet(0x82, 1, 0, &vec![2; n]), packet(0x85, 2, 7, &[3; 10])]
                .iter()
                .filter_map(|p| assembler.push(p))
                .collect();
        assert_eq!(frame.len(), 1);
        assert_eq!(frame[0].data.len(), 2 * n + 10);
        assert_eq!(frame[0].timestamp, 7);

        // Middle packets must still be full, and frames can't outgrow the maximum.
        assert!(assembler.push(&packet(0x81, 3, 0, &vec![1; n])).is_none());
        assert!(assembler.push(&packet(0x82, 4, 0, &[2; 5])).is_none());
        assert!(assembler.push(&packet(0x85, 5, 0, &[3; 5])).is_none());
        assert_eq!(assembler.dropped_frames(), 1);
    }

    #[test]
    fn unpacks_msb_first_samples() {
        // 11-bit samples 11111111111 00000000001 10000000000, padded: ff e0 06 00 00
        let raw = [0xff, 0xe0, 0x06, 0x00, 0x00];
        let mut out = [0u16; 3];
        unpack_bits(&raw, 11, &mut out);
        assert_eq!(out, [0x7ff, 0x001, 0x400]);

        // 10-bit samples 1000000001 0000000010 0000000100 0000000001
        let mut ten = [0u16; 4];
        unpack_bits(&[0b1000_0000, 0b0100_0000, 0b0010_0000, 0b0001_0000, 0b0000_0001], 10, &mut ten);
        assert_eq!(ten, [0b10_0000_0001, 2, 4, 1]);
    }
}
