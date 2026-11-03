//! Depth stream registers and frames, with no I/O.

use crate::stream::{RawFrame, unpack_bits};

pub const WIDTH: usize = 640;
pub const HEIGHT: usize = 480;
/// Raw value of pixels with no depth reading.
pub const NO_DEPTH: u16 = 2047;

pub const ENDPOINT: u8 = 0x82;
/// Bytes of one 640×480 frame of packed 11-bit samples.
pub const PACKED_FRAME_LEN: usize = WIDTH * HEIGHT * 11 / 8;

/// Register writes that start 640×480, 11-bit depth at 30 fps (from libfreenect).
pub const START: [(u16, u16); 7] = [
    (0x105, 0x00), // keep the projector on rather than cycling it
    (0x06, 0x00),  // stop depth
    (0x12, 0x03),  // 11-bit samples
    (0x13, 0x01),  // 640×480
    (0x14, 0x1e),  // 30 fps
    (0x06, 0x02),  // start depth
    (0x17, 0x00),  // not mirrored
];
pub const STOP: [(u16, u16); 1] = [(0x06, 0x00)];

/// One depth frame of raw 11-bit disparity values, row by row from the top left.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DepthFrame {
    pub data: Vec<u16>,
    pub timestamp: u32,
}

impl DepthFrame {
    pub fn unpack(raw: &RawFrame) -> Self {
        let mut data = vec![0; WIDTH * HEIGHT];
        unpack_bits(&raw.data, 11, &mut data);
        Self { data, timestamp: raw.timestamp }
    }

    pub fn get(&self, x: usize, y: usize) -> u16 {
        self.data[y * WIDTH + x]
    }
}
