//! Tilt motor, LED and accelerometer through the audio device (Kinect for Windows, model 1473).

use std::time::Duration;

use super::{UsbNode, WinUsbDevice, scan};
use crate::motor::{Led, TiltState, audio};
use crate::{Component, Error, Result};

const TIMEOUT: Duration = Duration::from_millis(250);

pub struct Motor {
    device: WinUsbDevice,
    tag: u32,
}

impl Motor {
    /// Opens the motor of the first connected Kinect.
    pub fn open() -> Result<Self> {
        let nodes = scan()?;
        let mut candidates = nodes.iter().filter(|n| n.component == Component::Audio && n.interface.unwrap_or(0) == 0);
        let first = candidates.clone().next().ok_or(Error::NotFound("audio device (which drives the motor)"))?;
        Self::open_node(candidates.find(|n| n.is_winusb()).unwrap_or(first))
    }

    /// Opens the motor through `node`, the audio device's first interface.
    pub fn open_node(node: &UsbNode) -> Result<Self> {
        let device = WinUsbDevice::open(node)?;
        for endpoint in [audio::OUT_ENDPOINT, audio::IN_ENDPOINT] {
            device.set_timeout(endpoint, TIMEOUT)?;
        }
        device.flush(audio::IN_ENDPOINT)?;
        Ok(Self { device, tag: 0 })
    }

    /// Sends a command and returns the data reply that precedes the acknowledgement, if any.
    fn send(&mut self, command: impl FnOnce(u32) -> Result<Vec<u8>>) -> Result<Vec<u8>> {
        let bytes = command(self.tag)?;
        self.tag = self.tag.wrapping_add(1);
        self.device.write(audio::OUT_ENDPOINT, &bytes)?;

        let mut buf = [0u8; 512];
        let len = self.device.read(audio::IN_ENDPOINT, &mut buf)?;
        if len == audio::ACK_LEN {
            audio::check_ack(&buf[..len])?;
            return Ok(Vec::new());
        }
        let reply = buf[..len].to_vec();
        let len = self.device.read(audio::IN_ENDPOINT, &mut buf)?;
        audio::check_ack(&buf[..len])?;
        Ok(reply)
    }

    pub fn state(&mut self) -> Result<TiltState> {
        let reply = self.send(|tag| Ok(audio::read_state(tag)))?;
        audio::parse_state(&reply)
    }

    /// Starts tilting to `degrees` from level. Returns before the motor has finished moving.
    pub fn set_tilt(&mut self, degrees: i8) -> Result<()> {
        self.send(|tag| audio::set_tilt(tag, degrees)).map(drop)
    }

    pub fn set_led(&mut self, led: Led) -> Result<()> {
        self.send(|tag| audio::set_led(tag, led)).map(drop)
    }
}
