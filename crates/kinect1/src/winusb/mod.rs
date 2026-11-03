//! WinUSB backend: talks to the Kinect directly with Windows' built-in WinUSB driver.
//!
//! The camera (and optionally the motor) must be bound to WinUSB, e.g. with Zadig.

mod device;
mod motor;
mod setup;

pub use device::WinUsbDevice;
pub use motor::Motor;
pub use setup::{UsbNode, scan};
