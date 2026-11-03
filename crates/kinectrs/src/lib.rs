//! Rust SDK for the Xbox 360 Kinect (Kinect v1): depth, color and IR streams, depth in
//! millimetres aligned with color, and the LED and accelerometer, through Windows' WinUSB driver.
//!
//! ```no_run
//! use kinectrs::Kinect;
//!
//! # fn main() -> kinectrs::Result<()> {
//! let kinect = Kinect::open()?;
//! let mut depth = kinect.start_depth()?;
//! let mut color = kinect.start_color()?;
//!
//! let depth_mm = depth.next_frame()?.to_mm(kinect.mm_table());
//! let rgb = color.next_frame()?.rgb;
//! // Depth on the color image's pixel grid, so depth_on_color[i] is the distance of rgb pixel i.
//! let depth_on_color = kinect.registration().depth_to_color(&depth_mm);
//! # Ok(())
//! # }
//! ```
//!
//! The camera must use the WinUSB driver; see the README for setting it up with Zadig.
//! [`winusb`] holds the device types. The other modules ([`camera`], [`depth`], [`registration`],
//! [`stream`], ...) are plain parsing and protocol code with no I/O.

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
#[cfg(windows)]
pub use winusb::Kinect;
