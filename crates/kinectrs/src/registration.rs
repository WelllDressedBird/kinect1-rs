//! Aligning depth with the color image ("registration"), with no I/O.
//!
//! The IR and color cameras sit a couple of centimetres apart, so a depth pixel and the color pixel
//! with the same coordinates see different points. The camera stores a polynomial model of the lens
//! differences; together with a depth pixel's distance it gives the color pixel that sees the same
//! point. This is a port of libfreenect's registration.c.

use crate::depth::{Calibration, HEIGHT, MAX_DEPTH_MM, NO_DEPTH_MM, WIDTH};
use crate::video::{COLOR_HEIGHT, COLOR_WIDTH};
use crate::{Error, Result};

/// Payload of the "algorithm parameters" command (0x16) asking for the registration model of
/// 640×480 color at 30 fps: parameter 0x40, format 0, resolution 1, 30 fps, offset 0.
pub const REG_INFO_PAYLOAD: [u8; 10] = [0x40, 0, 0, 0, 1, 0, 30, 0, 0, 0];
/// The same for the padding parameters (0x41).
pub const PAD_INFO_PAYLOAD: [u8; 10] = [0x41, 0, 0, 0, 1, 0, 30, 0, 0, 0];

/// Fixed-point scale of horizontal positions in the tables.
const X_SCALE: i32 = 256;
/// Constants of the depth-to-color shift, from libfreenect.
const S2D_PIXEL_CONST: f64 = 10.0;
const S2D_CONST_OFFSET: f64 = 0.375;
/// Horizontal resolution of the depth sensor, which 640-wide frames are scaled from.
const SENSOR_WIDTH: usize = 1280;
/// libfreenect, like OpenNI, uses this instead of the 2.3 cm the camera reports.
const COLOR_CAMERA_DISTANCE: f64 = 2.4;

/// The camera's polynomial model of where each depth pixel lands in the color image, as sent by
/// the camera. Fields the mapping does not use are skipped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RegInfo {
    pub ax: i32,
    pub bx: i32,
    pub cx: i32,
    pub dx: i32,
    pub dx_start: i32,
    pub ay: i32,
    pub by: i32,
    pub cy: i32,
    pub dy: i32,
    pub dy_start: i32,
    pub dxdx_start: i32,
    pub dxdy_start: i32,
    pub dydx_start: i32,
    pub dydy_start: i32,
    pub dxdxdx_start: i32,
    pub dydxdx_start: i32,
    pub dxdxdy_start: i32,
    pub dydxdy_start: i32,
    pub dydydx_start: i32,
    pub dydydy_start: i32,
}

impl RegInfo {
    const FIELDS: usize = 29;

    /// Parses the reply to [`REG_INFO_PAYLOAD`]: a status word, then 29 `i32`s.
    pub fn parse(reply: &[u8]) -> Result<Self> {
        if reply.len() < 2 + 4 * Self::FIELDS {
            return Err(Error::Protocol(format!("registration reply is {} bytes", reply.len())));
        }
        let f = |i: usize| i32::from_le_bytes(reply[2 + 4 * i..6 + 4 * i].try_into().unwrap());
        // Field order on the wire; dx_center, the beta and rollout fields and back_comp1/2 are unused.
        Ok(Self {
            ax: f(1),
            bx: f(2),
            cx: f(3),
            dx: f(4),
            dx_start: f(5),
            ay: f(6),
            by: f(7),
            cy: f(8),
            dy: f(9),
            dy_start: f(10),
            dxdx_start: f(17),
            dxdy_start: f(18),
            dydx_start: f(19),
            dydy_start: f(20),
            dxdxdx_start: f(21),
            dydxdx_start: f(22),
            dxdxdy_start: f(23),
            dydxdy_start: f(24),
            dydydx_start: f(26),
            dydydy_start: f(28),
        })
    }
}

/// Parses the reply to [`PAD_INFO_PAYLOAD`]: a status word, then start, end and cropping lines.
/// Returns the start lines, the only value the mapping uses.
pub fn parse_pad_start_lines(reply: &[u8]) -> Result<u16> {
    match reply {
        [_, _, lo, hi, ..] if reply.len() >= 8 => Ok(u16::from_le_bytes([*lo, *hi])),
        _ => Err(Error::Protocol(format!("registration padding reply is {} bytes", reply.len()))),
    }
}

/// Offsets of each depth pixel in the color image, from the camera's polynomial model, in pixels.
/// A port of libfreenect's freenect_create_dxdy_tables, which evaluates the polynomial by finite
/// differences in fixed point.
fn dxdy_tables(r: &RegInfo) -> (Vec<f64>, Vec<f64>) {
    let (ax6, bx6, cx2, dx2) = (i64::from(r.ax), i64::from(r.bx), i64::from(r.cx), i64::from(r.dx));
    let (ay6, by6, cy2, dy2) = (i64::from(r.ay), i64::from(r.by), i64::from(r.cy), i64::from(r.dy));

    // The left shifts must wrap at 32 bits before the right shifts sign-extend.
    let start = |v: i32, left: u32, right: u32| i64::from(v.wrapping_shl(left) >> right);
    let third = |v: i32| i64::from(v.wrapping_shl(5).wrapping_shl(3));
    let mut dx0 = start(r.dx_start, 13, 4);
    let mut dy0 = start(r.dy_start, 13, 4);
    let mut dxdx0 = start(r.dxdx_start, 11, 3);
    let mut dxdy0 = start(r.dxdy_start, 11, 3);
    let mut dydx0 = start(r.dydx_start, 11, 3);
    let mut dydy0 = start(r.dydy_start, 11, 3);
    let mut dxdxdx0 = third(r.dxdxdx_start);
    let mut dydxdx0 = third(r.dydxdx_start);
    let mut dydxdy0 = third(r.dydxdy_start);
    let mut dxdxdy0 = third(r.dxdxdy_start);
    let mut dydydx0 = third(r.dydydx_start);
    let mut dydydy0 = third(r.dydydy_start);

    let to_pixels = 1.0 / f64::from(1 << 17);
    let mut x_table = Vec::with_capacity(WIDTH * HEIGHT);
    let mut y_table = Vec::with_capacity(WIDTH * HEIGHT);
    for _row in 0..HEIGHT {
        dxdxdx0 += cx2;
        dxdx0 += dydxdx0 >> 8;
        dydxdx0 += dx2;
        dx0 += dydx0 >> 6;
        dydx0 += dydydx0 >> 8;
        dydydx0 += bx6;

        dxdxdy0 += cy2;
        dxdy0 += dydxdy0 >> 8;
        dydxdy0 += dy2;
        dy0 += dydy0 >> 6;
        dydy0 += dydydy0 >> 8;
        dydydy0 += by6;

        let (mut col_dxdxdy0, mut col_dxdy0, mut col_dy0) = (dxdxdy0, dxdy0, dy0);
        let (mut col_dxdxdx0, mut col_dxdx0, mut col_dx0) = (dxdxdx0, dxdx0, dx0);
        for _col in 0..WIDTH {
            x_table.push(col_dx0 as f64 * to_pixels);
            y_table.push(col_dy0 as f64 * to_pixels);

            col_dx0 += col_dxdx0 >> 6;
            col_dxdx0 += col_dxdxdx0 >> 8;
            col_dxdxdx0 += ax6;

            col_dy0 += col_dxdy0 >> 6;
            col_dxdy0 += col_dxdxdy0 >> 8;
            col_dxdxdy0 += ay6;
        }
    }
    (x_table, y_table)
}

/// Lookup tables mapping depth pixels to color pixels for one camera.
#[derive(Clone, Debug)]
pub struct Registration {
    /// Per depth pixel: color x in 1/256 pixels before the depth-dependent shift, and color y.
    /// x is out of range where the pixel has no counterpart.
    table: Vec<(i32, i32)>,
    /// Per distance in mm: horizontal shift in 1/256 pixels.
    shift: Vec<i32>,
    /// Rows the color image is offset by.
    start_lines: i32,
}

impl Registration {
    pub fn new(reg_info: &RegInfo, pad_start_lines: u16, calibration: &Calibration) -> Self {
        let (dx, dy) = dxdy_tables(reg_info);
        let mut table = Vec::with_capacity(WIDTH * HEIGHT);
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                let i = y * WIDTH + x;
                // libfreenect also moves every pixel one to the right and down.
                let mut new_x = x as f64 + dx[i] + 1.0;
                let new_y = y as f64 + dy[i] + 1.0;
                if new_x < 0.0 || new_y < 0.0 || new_x >= WIDTH as f64 || new_y >= HEIGHT as f64 {
                    new_x = 2.0 * WIDTH as f64;
                }
                table.push(((new_x * f64::from(X_SCALE)) as i32, new_y as i32));
            }
        }

        let x_scale = (SENSOR_WIDTH / WIDTH) as f64;
        let pixel_size = 1.0 / (calibration.reference_pixel_size * x_scale * S2D_PIXEL_CONST);
        let camera_distance = COLOR_CAMERA_DISTANCE * pixel_size * S2D_PIXEL_CONST;
        let reference_distance = calibration.reference_distance * pixel_size * S2D_PIXEL_CONST;
        let shift = (0..=MAX_DEPTH_MM)
            .map(|mm| {
                let depth = f64::from(mm) * pixel_size;
                let pixels = camera_distance * (depth - reference_distance) / depth + S2D_CONST_OFFSET;
                (pixels * f64::from(X_SCALE)) as i32
            })
            .collect();

        Self { table, shift, start_lines: i32::from(pad_start_lines) }
    }

    /// The color pixel that sees the same point as depth pixel (x, y) at `mm` millimetres, if
    /// it is inside the color image.
    pub fn color_pixel(&self, x: usize, y: usize, mm: u16) -> Option<(usize, usize)> {
        if mm == NO_DEPTH_MM || mm > MAX_DEPTH_MM || x >= WIDTH || y >= HEIGHT {
            return None;
        }
        let (table_x, table_y) = self.table[y * WIDTH + x];
        let cx = (table_x + self.shift[usize::from(mm)]) / X_SCALE;
        let cy = table_y - self.start_lines;
        let inside = (0..COLOR_WIDTH as i32).contains(&cx) && (0..COLOR_HEIGHT as i32).contains(&cy);
        inside.then_some((cx as usize, cy as usize))
    }

    /// Re-projects a depth image in millimetres onto the color image's pixel grid: the result's
    /// pixel (x, y) is the distance seen by color pixel (x, y), or [`NO_DEPTH_MM`]. Where several
    /// depth pixels land on one color pixel, the nearest wins.
    pub fn depth_to_color(&self, depth_mm: &[u16]) -> Vec<u16> {
        let mut out = vec![NO_DEPTH_MM; COLOR_WIDTH * COLOR_HEIGHT];
        for (i, &mm) in depth_mm.iter().enumerate().take(WIDTH * HEIGHT) {
            if let Some((cx, cy)) = self.color_pixel(i % WIDTH, i / WIDTH, mm) {
                let target = &mut out[cy * COLOR_WIDTH + cx];
                if *target == NO_DEPTH_MM || *target > mm {
                    *target = mm;
                }
            }
        }
        out
    }

    /// The color of each depth pixel: RGB triples on the depth image's grid, black where there is
    /// no depth, the point lies outside the color image, or it is hidden from the color camera
    /// by something nearer.
    pub fn color_for_depth(&self, depth_mm: &[u16], rgb: &[u8]) -> Vec<u8> {
        let nearest = self.depth_to_color(depth_mm);
        let mut out = vec![0; WIDTH * HEIGHT * 3];
        for (i, &mm) in depth_mm.iter().enumerate().take(WIDTH * HEIGHT) {
            let Some((cx, cy)) = self.color_pixel(i % WIDTH, i / WIDTH, mm) else { continue };
            let c = cy * COLOR_WIDTH + cx;
            if mm <= nearest[c] {
                out[3 * i..3 * i + 3].copy_from_slice(&rgb[3 * c..3 * c + 3]);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn parses_reg_info_in_wire_order() {
        let mut reply = vec![0u8; 118];
        for i in 0..RegInfo::FIELDS {
            reply[2 + 4 * i..6 + 4 * i].copy_from_slice(&(i as i32 * 10).to_le_bytes());
        }
        let r = RegInfo::parse(&reply).unwrap();
        assert_eq!((r.ax, r.dx_start, r.ay, r.dy_start), (10, 50, 60, 100));
        assert_eq!((r.dxdx_start, r.dydydx_start, r.dydydy_start), (170, 260, 280));
        assert!(RegInfo::parse(&reply[..117]).is_err());
        assert_eq!(parse_pad_start_lines(&[0, 0, 4, 0, 0, 0, 0, 0]).unwrap(), 4);
    }

    #[test]
    fn identity_model_shifts_by_distance_only() {
        // With an all-zero polynomial each depth pixel maps to itself (plus libfreenect's one
        // pixel offset) and then shifts horizontally with distance.
        let reg = Registration::new(&RegInfo::default(), 0, &calibration());
        let (near_x, near_y) = reg.color_pixel(300, 200, 600).unwrap();
        let (far_x, far_y) = reg.color_pixel(300, 200, 3000).unwrap();
        assert_eq!((near_y, far_y), (201, 201));
        assert!(near_x < far_x, "near objects shift further: {near_x} vs {far_x}");
        // At the reference plane (1200 mm) the shift is only the constant 0.375 px.
        assert_eq!(reg.color_pixel(300, 200, 1200).unwrap(), (301, 201));
        assert_eq!(reg.color_pixel(300, 200, NO_DEPTH_MM), None);
        assert_eq!(reg.color_pixel(639, 479, 1200), None, "pushed off the bottom right");
    }

    #[test]
    fn keeps_the_nearest_depth_and_hides_occluded_colour() {
        let reg = Registration::new(&RegInfo::default(), 0, &calibration());
        let mut depth_mm = vec![NO_DEPTH_MM; WIDTH * HEIGHT];
        let (a, b) = (200 * WIDTH + 300, 200 * WIDTH + 301);
        depth_mm[a] = 1200;
        depth_mm[b] = 1200;
        let registered = reg.depth_to_color(&depth_mm);
        assert_eq!(registered[201 * WIDTH + 301], 1200);
        assert_eq!(registered[201 * WIDTH + 302], 1200);
        assert_eq!(registered.iter().filter(|&&d| d != NO_DEPTH_MM).count(), 2);

        let mut rgb = vec![0u8; COLOR_WIDTH * COLOR_HEIGHT * 3];
        rgb[3 * (201 * WIDTH + 301)..][..3].copy_from_slice(&[9, 8, 7]);
        let colours = reg.color_for_depth(&depth_mm, &rgb);
        assert_eq!(&colours[3 * a..3 * a + 3], [9, 8, 7]);
    }

    #[test]
    fn polynomial_start_values_wrap_at_32_bits() {
        // dx_start << 13 overflows i32; libfreenect relies on the wrap before shifting back.
        let r = RegInfo { dx_start: 0x0004_0001, ..Default::default() };
        let (dx, _) = dxdy_tables(&r);
        let expected = f64::from(0x0004_0001i32.wrapping_shl(13) >> 4) / f64::from(1 << 17);
        assert_eq!(dx[0], expected);
    }
}
