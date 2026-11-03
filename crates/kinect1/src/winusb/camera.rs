//! Commands and registers of the depth/color camera.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use super::isoch::IsochReader;
use super::{UsbNode, WinUsbDevice, scan};
use crate::depth::{self, DepthFrame};
use crate::stream::{FrameAssembler, RawFrame, StreamKind};
use crate::{Component, Error, Result, camera};

const VENDOR_OUT: u8 = 0x40;
const VENDOR_IN: u8 = 0xc0;
const REPLY_TIMEOUT: Duration = Duration::from_secs(1);

/// Isochronous transfers kept queued per stream, and packets (125 µs microframes) in each.
const TRANSFERS: usize = 8;
const PACKETS_PER_TRANSFER: usize = 64;
const FRAME_TIMEOUT: Duration = Duration::from_secs(2);

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
                return Err(Error::Timeout("a camera command reply"));
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

    /// Starts the depth stream: 640×480 raw 11-bit values at 30 fps. It stops when dropped.
    pub fn start_depth(&mut self) -> Result<DepthStream<'_>> {
        let endpoint = self
            .device
            .configuration()?
            .settings
            .iter()
            .flat_map(|s| &s.endpoints)
            .find(|e| e.address == depth::ENDPOINT)
            .cloned()
            .ok_or_else(|| Error::Protocol("camera has no depth endpoint".into()))?;
        let reader = IsochReader::new(
            &self.device,
            depth::ENDPOINT,
            endpoint.bytes_per_interval() as usize,
            TRANSFERS,
            PACKETS_PER_TRANSFER,
        )?;
        let stream = DepthStream {
            camera: self,
            reader,
            assembler: FrameAssembler::new(StreamKind::Depth, depth::PACKED_FRAME_LEN),
            ready: VecDeque::new(),
        };
        for (register, value) in depth::START {
            stream.camera.write_register(register, value)?;
        }
        Ok(stream)
    }
}

pub struct DepthStream<'a> {
    camera: &'a mut Camera,
    reader: IsochReader,
    assembler: FrameAssembler,
    ready: VecDeque<RawFrame>,
}

impl DepthStream<'_> {
    /// Waits for the next complete frame.
    pub fn next_frame(&mut self) -> Result<DepthFrame> {
        let start = Instant::now();
        loop {
            if let Some(raw) = self.ready.pop_front() {
                return Ok(DepthFrame::unpack(&raw));
            }
            if start.elapsed() > FRAME_TIMEOUT {
                return Err(Error::Timeout("a depth frame"));
            }
            let (assembler, ready) = (&mut self.assembler, &mut self.ready);
            self.reader.read(FRAME_TIMEOUT, |packet| ready.extend(assembler.push(packet)))?;
        }
    }

    /// Frames lost so far to dropped or out-of-order packets.
    pub fn dropped_frames(&self) -> u64 {
        self.assembler.dropped_frames()
    }
}

impl Drop for DepthStream<'_> {
    fn drop(&mut self) {
        for (register, value) in depth::STOP {
            let _ = self.camera.write_register(register, value);
        }
    }
}
