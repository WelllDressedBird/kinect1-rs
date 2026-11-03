//! Color and IR streams: registers and frames, with no I/O.
//!
//! Both come from the same image endpoint, so only one of them can run at a time.

use crate::stream::{RawFrame, unpack_bits};

pub const ENDPOINT: u8 = 0x81;

/// Size of a color frame at [`Resolution::Medium`], which depth registration maps onto.
pub const COLOR_WIDTH: usize = 640;
pub const COLOR_HEIGHT: usize = 480;

/// Size of an IR frame at [`Resolution::Medium`]: eight rows taller than the color one.
pub const IR_WIDTH: usize = 640;
pub const IR_HEIGHT: usize = 488;

/// Image size of the color and IR streams.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Resolution {
    /// 640×480 color or 640×488 IR, at 30 fps.
    #[default]
    Medium,
    /// 1280×1024 IR, and 1280×1024 or 1280×960 color depending on the Kinect model (a model
    /// 1473 sends 960 rows; the frame says which), at up to 15 fps. High-resolution IR cannot run
    /// alongside depth.
    High,
}

impl Resolution {
    /// Width and largest height of a color frame.
    pub fn color_size(self) -> (usize, usize) {
        match self {
            Self::Medium => (COLOR_WIDTH, COLOR_HEIGHT),
            Self::High => (1280, 1024),
        }
    }

    /// Width and height of an IR frame.
    pub fn ir_size(self) -> (usize, usize) {
        match self {
            Self::Medium => (IR_WIDTH, IR_HEIGHT),
            Self::High => (1280, 1024),
        }
    }

    /// Bytes of the largest frame of Bayer samples, one byte per pixel.
    pub fn bayer_frame_len(self) -> usize {
        let (w, h) = self.color_size();
        w * h
    }

    /// Bytes of one frame of packed 10-bit IR samples.
    pub fn ir_packed_frame_len(self) -> usize {
        let (w, h) = self.ir_size();
        w * h * 10 / 8
    }

    /// Register writes that start Bayer color (from libfreenect).
    pub fn color_start(self) -> &'static [(u16, u16)] {
        match self {
            Self::Medium => &[
                (0x0c, 0x00), // Bayer
                (0x0d, 0x01), // 640×480
                (0x0e, 0x1e), // 30 fps
                (0x05, 0x01), // start color
                (0x47, 0x00), // not mirrored
            ],
            Self::High => &[
                (0x0c, 0x00), // Bayer
                (0x0d, 0x02), // 1280×1024
                (0x0e, 0x0f), // 15 fps
                (0x05, 0x01), // start color
                (0x47, 0x00), // not mirrored
            ],
        }
    }

    /// Register writes that start 10-bit IR (from libfreenect).
    pub fn ir_start(self) -> &'static [(u16, u16)] {
        match self {
            Self::Medium => &[
                (0x19, 0x00),  // 10-bit
                (0x1a, 0x01),  // 640×488
                (0x1b, 0x1e),  // 30 fps
                (0x105, 0x00), // keep the projector on rather than cycling it
                (0x05, 0x03),  // start IR
                (0x48, 0x00),  // not mirrored
            ],
            Self::High => &[
                // The firmware only hands out 1280×1024 IR after depth has run once.
                (0x13, 0x01),  // depth 640×480
                (0x14, 0x1e),  // depth 30 fps
                (0x06, 0x02),  // start depth
                (0x06, 0x00),  // stop depth
                (0x19, 0x00),  // 10-bit
                (0x1a, 0x02),  // 1280×1024
                (0x1b, 0x0f),  // 15 fps
                (0x105, 0x00), // keep the projector on rather than cycling it
                (0x05, 0x03),  // start IR
                (0x48, 0x00),  // not mirrored
            ],
        }
    }
}

pub const STOP: [(u16, u16); 1] = [(0x05, 0x00)];

/// One color frame, 8-bit RGB triples row by row from the top left.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ColorFrame {
    pub width: usize,
    pub height: usize,
    pub rgb: Vec<u8>,
    pub timestamp: u32,
}

impl ColorFrame {
    pub fn from_bayer(raw: &RawFrame, resolution: Resolution) -> Self {
        let width = resolution.color_size().0;
        let height = raw.data.len() / width;
        Self { width, height, rgb: demosaic(&raw.data, width, height), timestamp: raw.timestamp }
    }

    pub fn get(&self, x: usize, y: usize) -> [u8; 3] {
        let i = 3 * (y * self.width + x);
        [self.rgb[i], self.rgb[i + 1], self.rgb[i + 2]]
    }
}

/// One IR frame of 10-bit intensities, row by row from the top left.
///
/// The camera does not timestamp IR packets (a Kinect for Windows sends the same value for every
/// frame), so `timestamp` is only meaningful for telling streams apart, not frames.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IrFrame {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u16>,
    pub timestamp: u32,
}

impl IrFrame {
    pub fn unpack(raw: &RawFrame, resolution: Resolution) -> Self {
        let (width, height) = resolution.ir_size();
        let mut data = vec![0; width * height];
        unpack_bits(&raw.data, 10, &mut data);
        Self { width, height, data, timestamp: raw.timestamp }
    }

    pub fn get(&self, x: usize, y: usize) -> u16 {
        self.data[y * self.width + x]
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
        assert_eq!(Resolution::Medium.bayer_frame_len(), 307_200);
        assert_eq!(Resolution::Medium.ir_packed_frame_len(), 390_400);
        assert_eq!(Resolution::High.bayer_frame_len(), 1_310_720);
        assert_eq!(Resolution::High.ir_packed_frame_len(), 1_638_400);
    }

    #[test]
    fn high_resolution_ir_cycles_depth_first() {
        let start = Resolution::High.ir_start();
        assert_eq!(&start[..4], [(0x13, 1), (0x14, 0x1e), (0x06, 2), (0x06, 0)]);
        assert!(start.contains(&(0x1a, 0x02)));
        assert!(Resolution::High.color_start().contains(&(0x0d, 0x02)));
    }
}
