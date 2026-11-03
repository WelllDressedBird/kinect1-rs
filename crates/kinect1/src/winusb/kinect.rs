//! The whole sensor behind one handle.

use super::{Camera, ColorStream, DepthStream, IrStream, Motor};
use crate::Result;
use crate::depth::{Calibration, MmTable};
use crate::registration::Registration;
use crate::video::Resolution;

/// A connected Kinect: its camera with the calibration read from it, and its motor when it can be
/// reached.
pub struct Kinect {
    camera: Camera,
    motor: Option<Motor>,
    calibration: Calibration,
    mm_table: MmTable,
    registration: Registration,
}

impl Kinect {
    /// Opens the first connected Kinect and reads its calibration. Fails if the camera can't be
    /// opened; a motor that can't be opened only leaves [`Kinect::motor`] empty.
    pub fn open() -> Result<Self> {
        let camera = Camera::open()?;
        let calibration = camera.depth_calibration()?;
        let registration = camera.registration()?;
        Ok(Self { motor: Motor::open().ok(), mm_table: calibration.mm_table(), calibration, registration, camera })
    }

    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    /// The tilt motor, LED and accelerometer, or `None` if they could not be opened when the
    /// Kinect was. [`Motor::open`] reports why.
    pub fn motor(&self) -> Option<&Motor> {
        self.motor.as_ref()
    }

    pub fn calibration(&self) -> &Calibration {
        &self.calibration
    }

    /// Converts raw depth to millimetres; see [`crate::depth::DepthFrame::to_mm`].
    pub fn mm_table(&self) -> &MmTable {
        &self.mm_table
    }

    /// Aligns depth with the color image.
    pub fn registration(&self) -> &Registration {
        &self.registration
    }

    /// See [`Camera::start_depth`].
    pub fn start_depth(&self) -> Result<DepthStream<'_>> {
        self.camera.start_depth()
    }

    /// See [`Camera::start_color`].
    pub fn start_color(&self) -> Result<ColorStream<'_>> {
        self.camera.start_color()
    }

    /// See [`Camera::start_ir`].
    pub fn start_ir(&self) -> Result<IrStream<'_>> {
        self.camera.start_ir()
    }

    /// See [`Camera::start_color_at`].
    pub fn start_color_at(&self, resolution: Resolution) -> Result<ColorStream<'_>> {
        self.camera.start_color_at(resolution)
    }

    /// See [`Camera::start_ir_at`].
    pub fn start_ir_at(&self, resolution: Resolution) -> Result<IrStream<'_>> {
        self.camera.start_ir_at(resolution)
    }
}
