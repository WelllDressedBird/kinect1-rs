//! Depth stream registers, frames and conversion to millimetres, with no I/O.

use crate::stream::{RawFrame, unpack_bits};
use crate::{Error, Result};

pub const WIDTH: usize = 640;
pub const HEIGHT: usize = 480;
/// Raw value of pixels with no depth reading.
pub const NO_DEPTH: u16 = 2047;
/// Millimetre value of pixels with no depth reading.
pub const NO_DEPTH_MM: u16 = 0;
/// Readings beyond this many millimetres are treated as no reading.
pub const MAX_DEPTH_MM: u16 = 10_000;

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

/// IR projector brightness register, 1..=50; Kinects start at 30.
pub const IR_BRIGHTNESS_REGISTER: u16 = 0x15;
pub const MAX_IR_BRIGHTNESS: u16 = 50;
/// Register writes for Kinect for Windows near mode (from libfreenect): a dimmer projector and
/// a different range setting. They need a pause between them.
pub const NEAR_MODE_ON: [(u16, u16); 2] = [(IR_BRIGHTNESS_REGISTER, 0x07), (0x2ef, 0x0000)];
pub const NEAR_MODE_OFF: [(u16, u16); 2] = [(IR_BRIGHTNESS_REGISTER, 0x1e), (0x2ef, 0x0190)];
/// Product id of Kinect for Windows cameras, the only ones with near mode.
pub const K4W_CAMERA_PID: u16 = 0x02bf;

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

    /// Distances in millimetres, [`NO_DEPTH_MM`] where there is no reading.
    pub fn to_mm(&self, table: &MmTable) -> Vec<u16> {
        self.data.iter().map(|&raw| table.mm(raw)).collect()
    }
}

/// Payload of the camera's "fixed parameters" command (0x04): a zero offset.
pub const FIXED_PARAMS_PAYLOAD: [u8; 10] = [0; 10];
/// Payload of the "algorithm parameters" command (0x16) asking for the constant shift of
/// 640×480 depth at 30 fps: parameter 0, format 0, resolution 1, 30 fps, offset 0.
pub const CONST_SHIFT_PAYLOAD: [u8; 10] = [0, 0, 0, 0, 1, 0, 30, 0, 0, 0];

/// Where the zero-plane block starts in the fixed parameters reply.
const ZERO_PLANE_OFFSET: usize = 94;

/// Per-device depth calibration, as stored in the camera.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
    /// Distance between the IR camera and the IR projector, in cm.
    pub emitter_distance: f64,
    /// Distance between the IR camera and the color camera, in cm.
    pub color_camera_distance: f64,
    /// Distance of the reference plane the disparity is measured against.
    pub reference_distance: f64,
    /// Size of one pixel on the reference plane, in mm, for the 1280×1024 sensor.
    pub reference_pixel_size: f64,
    /// Offset of the raw disparity values.
    pub const_shift: f64,
}

impl Calibration {
    /// Builds the calibration from the payloads of the fixed parameters and constant shift
    /// replies.
    pub fn parse(fixed_params: &[u8], const_shift: &[u8]) -> Result<Self> {
        let block = fixed_params
            .get(ZERO_PLANE_OFFSET..ZERO_PLANE_OFFSET + 16)
            .ok_or_else(|| Error::Protocol(format!("fixed parameters reply is {} bytes", fixed_params.len())))?;
        let float = |i: usize| f64::from(f32::from_le_bytes(block[4 * i..4 * i + 4].try_into().unwrap()));
        let shift = match const_shift {
            [_, _, lo, hi, ..] => u16::from_le_bytes([*lo, *hi]),
            _ => return Err(Error::Protocol(format!("constant shift reply is {} bytes", const_shift.len()))),
        };
        let calibration = Self {
            emitter_distance: float(0),
            color_camera_distance: float(1),
            reference_distance: float(2),
            reference_pixel_size: float(3),
            const_shift: f64::from(shift),
        };
        let usable = [calibration.emitter_distance, calibration.reference_distance, calibration.reference_pixel_size]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0);
        if !usable {
            return Err(Error::Protocol(format!("implausible depth calibration {calibration:?}")));
        }
        Ok(calibration)
    }

    /// Converts one raw disparity value to millimetres (libfreenect's formula), or `None` when
    /// there is no reading or it lies outside 0..=[`MAX_DEPTH_MM`].
    pub fn raw_to_mm(&self, raw: u16) -> Option<u16> {
        if raw == NO_DEPTH {
            return None;
        }
        let shifted = (f64::from(raw) - 4.0 * self.const_shift) / 4.0 - 0.375;
        let metric = shifted * self.reference_pixel_size;
        let mm = 10.0 * (metric * self.reference_distance / (self.emitter_distance - metric) + self.reference_distance);
        (mm > 0.0 && mm <= f64::from(MAX_DEPTH_MM)).then(|| mm.round() as u16)
    }

    /// Position in millimetres of the depth pixel (x, y) at `mm` from the camera: x to the right,
    /// y down, z forward, with the origin at the IR camera.
    pub fn camera_to_world(&self, x: usize, y: usize, mm: u16) -> [f64; 3] {
        // A 640×480 frame is the 1280×1024 sensor cropped to 1280×960 and halved.
        let factor = 2.0 * self.reference_pixel_size * f64::from(mm) / self.reference_distance;
        [(x as f64 - WIDTH as f64 / 2.0) * factor, (y as f64 - HEIGHT as f64 / 2.0) * factor, f64::from(mm)]
    }

    pub fn mm_table(&self) -> MmTable {
        MmTable((0..=NO_DEPTH).map(|raw| self.raw_to_mm(raw).unwrap_or(NO_DEPTH_MM)).collect())
    }
}

/// Raw disparity to millimetre lookup table, built once from a [`Calibration`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MmTable(Vec<u16>);

impl MmTable {
    pub fn mm(&self, raw: u16) -> u16 {
        self.0.get(usize::from(raw)).copied().unwrap_or(NO_DEPTH_MM)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Values typical of a real Kinect.
    fn calibration() -> Calibration {
        Calibration {
            emitter_distance: 7.5,
            color_camera_distance: 2.3,
            reference_distance: 120.0,
            reference_pixel_size: 0.1042,
            const_shift: 200.0,
        }
    }

    #[test]
    fn parses_calibration_replies() {
        let mut fixed = vec![0u8; 332];
        for (i, v) in [7.5f32, 2.3, 120.0, 0.1042].into_iter().enumerate() {
            fixed[94 + 4 * i..98 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        let c = Calibration::parse(&fixed, &[0, 0, 200, 0]).unwrap();
        assert_eq!(c.const_shift, 200.0);
        assert!((c.reference_pixel_size - 0.1042).abs() < 1e-6);

        assert!(Calibration::parse(&fixed[..100], &[0, 0, 200, 0]).is_err(), "truncated");
        assert!(Calibration::parse(&fixed, &[0, 0]).is_err(), "no shift");
        assert!(Calibration::parse(&[0; 332], &[0, 0, 200, 0]).is_err(), "all zero");
    }

    #[test]
    fn converts_raw_disparity_to_millimetres() {
        let c = calibration();
        // Agrees with the widely used fitted approximation 1 / (3.3309 - 0.0030711 * raw) m
        // to within the spread between real devices.
        for raw in [450u16, 700, 900, 1005] {
            let fitted = 1000.0 / (3.330_949_516_1 - 0.003_071_101_6 * f64::from(raw));
            let mm = f64::from(c.raw_to_mm(raw).unwrap());
            assert!((mm - fitted).abs() / fitted < 0.1, "raw {raw}: {mm} mm vs {fitted:.0} mm");
        }
        assert_eq!(c.raw_to_mm(NO_DEPTH), None);
        assert_eq!(c.raw_to_mm(1080), None, "beyond 10 m");

        let table = c.mm_table();
        assert_eq!(table.mm(1005), c.raw_to_mm(1005).unwrap());
        assert_eq!(table.mm(NO_DEPTH), NO_DEPTH_MM);
    }

    #[test]
    fn projects_pixels_into_space() {
        let c = calibration();
        assert_eq!(c.camera_to_world(320, 240, 2000), [0.0, 0.0, 2000.0]);
        let [x, y, _] = c.camera_to_world(640, 0, 2000);
        // About 28.5° to each side of a 57° field of view.
        assert!((x / 2000.0 - 0.556).abs() < 0.01, "{x}");
        assert!(y < 0.0);
    }
}
