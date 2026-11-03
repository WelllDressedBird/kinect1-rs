//! Color and IR streams: registers and frames, with no I/O.
//!
//! Both come from the same image endpoint, so only one of them can run at a time.

use crate::stream::{RawFrame, unpack_bits};

pub const ENDPOINT: u8 = 0x81;

pub const COLOR_WIDTH: usize = 640;
pub const COLOR_HEIGHT: usize = 480;
/// Bytes of one 640×480 frame of Bayer samples, one byte per pixel.
pub const BAYER_FRAME_LEN: usize = COLOR_WIDTH * COLOR_HEIGHT;

/// The IR image is eight rows taller than the color one.
pub const IR_WIDTH: usize = 640;
pub const IR_HEIGHT: usize = 488;
/// Bytes of one 640×488 frame of packed 10-bit samples.
pub const IR_PACKED_FRAME_LEN: usize = IR_WIDTH * IR_HEIGHT * 10 / 8;

/// Register writes that start 640×480 Bayer color at 30 fps (from libfreenect).
pub const COLOR_START: [(u16, u16); 5] = [
    (0x0c, 0x00), // Bayer
    (0x0d, 0x01), // 640×480
    (0x0e, 0x1e), // 30 fps
    (0x05, 0x01), // start color
    (0x47, 0x00), // not mirrored
];

/// Register writes that start 640×488 10-bit IR at 30 fps (from libfreenect).
pub const IR_START: [(u16, u16); 6] = [
    (0x19, 0x00),  // 10-bit
    (0x1a, 0x01),  // 640×488
    (0x1b, 0x1e),  // 30 fps
    (0x105, 0x00), // keep the projector on rather than cycling it
    (0x05, 0x03),  // start IR
    (0x48, 0x00),  // not mirrored
];

pub const STOP: [(u16, u16); 1] = [(0x05, 0x00)];

/// One color frame, 8-bit RGB triples row by row from the top left.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ColorFrame {
    pub rgb: Vec<u8>,
    pub timestamp: u32,
}

impl ColorFrame {
    pub fn from_bayer(raw: &RawFrame) -> Self {
        Self { rgb: demosaic(&raw.data, COLOR_WIDTH, COLOR_HEIGHT), timestamp: raw.timestamp }
    }

    pub fn get(&self, x: usize, y: usize) -> [u8; 3] {
        let i = 3 * (y * COLOR_WIDTH + x);
        [self.rgb[i], self.rgb[i + 1], self.rgb[i + 2]]
    }
}

/// One IR frame of 10-bit intensities, row by row from the top left.
///
/// The camera does not timestamp IR packets (a Kinect for Windows sends the same value for every
/// frame), so `timestamp` is only meaningful for telling streams apart, not frames.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IrFrame {
    pub data: Vec<u16>,
    pub timestamp: u32,
}

impl IrFrame {
    pub fn unpack(raw: &RawFrame) -> Self {
        let mut data = vec![0; IR_WIDTH * IR_HEIGHT];
        unpack_bits(&raw.data, 10, &mut data);
        Self { data, timestamp: raw.timestamp }
    }

    pub fn get(&self, x: usize, y: usize) -> u16 {
        self.data[y * IR_WIDTH + x]
    }
}

/// Which color the sensor samples at (x, y). The Kinect's filter pattern is
/// G R
/// B G
fn bayer_channel(x: usize, y: usize) -> usize {
    match (y % 2, x % 2) {
        (0, 1) => 0,
        (1, 0) => 2,
        _ => 1,
    }
}

/// Bilinear demosaicing: each channel of each pixel is the mean of the samples of that color in
/// its 3×3 neighbourhood.
pub fn demosaic(bayer: &[u8], width: usize, height: usize) -> Vec<u8> {
    let mut rgb = vec![0; width * height * 3];
    for y in 0..height {
        for x in 0..width {
            let mut sum = [0u32; 3];
            let mut count = [0u32; 3];
            for ny in y.saturating_sub(1)..=(y + 1).min(height - 1) {
                for nx in x.saturating_sub(1)..=(x + 1).min(width - 1) {
                    let c = bayer_channel(nx, ny);
                    sum[c] += u32::from(bayer[ny * width + nx]);
                    count[c] += 1;
                }
            }
            for c in 0..3 {
                rgb[3 * (y * width + x) + c] = (sum[c] / count[c].max(1)) as u8;
            }
        }
    }
    rgb
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Bayer mosaic of a flat colour.
    fn mosaic(width: usize, height: usize, colour: [u8; 3]) -> Vec<u8> {
        (0..height).flat_map(|y| (0..width).map(move |x| colour[bayer_channel(x, y)])).collect()
    }

    #[test]
    fn demosaics_flat_colours_exactly() {
        let rgb = demosaic(&mosaic(6, 4, [200, 100, 30]), 6, 4);
        assert!(rgb.chunks(3).all(|p| p == [200, 100, 30]), "{rgb:?}");
    }

    #[test]
    fn demosaic_keeps_each_sample_in_its_own_channel() {
        let mut bayer = vec![0u8; 4 * 4];
        bayer[1] = 90; // red sample at (1, 0)
        let rgb = demosaic(&bayer, 4, 4);
        assert_eq!(&rgb[3..6], [90, 0, 0]);
    }

    #[test]
    fn stream_sizes_match_libfreenect() {
        assert_eq!(BAYER_FRAME_LEN, 307_200);
        assert_eq!(IR_PACKED_FRAME_LEN, 390_400);
    }
}
