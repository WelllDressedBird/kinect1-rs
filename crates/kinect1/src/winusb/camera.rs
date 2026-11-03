//! Commands and registers of the depth/color camera.

use std::time::{Duration, Instant};

use super::{UsbNode, WinUsbDevice, scan};
use crate::camera;
use crate::{Component, Error, Result};

const VENDOR_OUT: u8 = 0x40;
const VENDOR_IN: u8 = 0xc0;
const REPLY_TIMEOUT: Duration = Duration::from_secs(1);

pub struct Camera {
    device: WinUsbDevice,
    tag: u16,
}

impl Camera {
    /// Opens the camera of the first connected Kinect.
    pub fn open() -> Result<Self> {
        let nodes = scan()?;
        let mut cameras = nodes.iter().filter(|n| n.component == Component::Camera);
        let first = cameras.clone().next().ok_or(Error::NotFound("camera"))?;
        Self::open_node(cameras.find(|n| n.is_winusb()).unwrap_or(first))
    }

    pub fn open_node(node: &UsbNode) -> Result<Self> {
        Ok(Self { device: WinUsbDevice::open(node)?, tag: 0 })
    }

    pub fn device(&self) -> &WinUsbDevice {
        &self.device
    }

    /// Sends `command` and returns the payload of the camera's reply.
    pub fn command(&mut self, command: u16, payload: &[u8]) -> Result<Vec<u8>> {
        let tag = self.tag;
        self.tag = self.tag.wrapping_add(1);
        self.device.control_out(VENDOR_OUT, 0, 0, 0, &camera::encode(command, tag, payload)?)?;

        // The camera answers "not ready" with an empty or completely full read.
        let start = Instant::now();
        let mut buf = [0u8; camera::REPLY_BUFFER_LEN];
        loop {
            let len = self.device.control_in(VENDOR_IN, 0, 0, 0, &mut buf)?;
            if len != 0 && len != buf.len() {
                return camera::decode_reply(command, tag, &buf[..len]).map(<[u8]>::to_vec);
            }
            if start.elapsed() > REPLY_TIMEOUT {
                return Err(Error::Protocol(format!("camera did not reply to command {command:#x}")));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    pub fn read_register(&mut self, register: u16) -> Result<u16> {
        let reply = self.command(camera::READ_REGISTER, &camera::read_register_payload(register))?;
        camera::parse_read_register(&reply)
    }

    pub fn write_register(&mut self, register: u16, value: u16) -> Result<()> {
        let reply = self.command(camera::WRITE_REGISTER, &camera::write_register_payload(register, value))?;
        camera::check_write_register(&reply)
    }
}
