//! Rust SDK for the Xbox 360 Kinect (Kinect v1).
//!
//! The `winusb` module talks to the sensor directly through Windows' WinUSB driver.
//! The other modules ([`camera`], [`depth`], [`stream`], ...) are plain parsing and protocol code with no I/O.

pub mod camera;
pub mod depth;
pub mod descriptors;
mod error;
pub mod ids;
pub mod motor;
pub mod registration;
pub mod stream;
pub mod video;
#[cfg(windows)]
pub mod winusb;

pub use error::{Error, Result};
pub use ids::Component;
