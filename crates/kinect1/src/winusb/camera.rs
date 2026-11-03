//! Commands, registers and streams of the depth/color camera.

use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use super::isoch::IsochReader;
use super::{UsbNode, WinUsbDevice, scan};
use crate::depth::{self, DepthFrame};
use crate::stream::{FrameAssembler, RawFrame, StreamKind};
use crate::video::{self, ColorFrame, IrFrame};
use crate::{Component, Error, Result, camera};

const VENDOR_OUT: u8 = 0x40;
const VENDOR_IN: u8 = 0xc0;
const REPLY_TIMEOUT: Duration = Duration::from_secs(1);

/// Isochronous transfers kept queued per stream, and packets (125 µs microframes) in each.
const TRANSFERS: usize = 8;
const PACKETS_PER_TRANSFER: usize = 64;
const FRAME_TIMEOUT: Duration = Duration::from_secs(2);

/// The camera. Commands may be sent from several threads, and the depth stream can run alongside
/// either the color or the IR stream.
pub struct Camera {
    device: WinUsbDevice,
    /// Tag of the next command. Held for a whole command so request/reply pairs don't interleave.
    tag: Mutex<u16>,
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
        Ok(Self { device: WinUsbDevice::open(node)?, tag: Mutex::new(0) })
    }

    pub fn device(&self) -> &WinUsbDevice {
        &self.device
    }

    /// Sends `command` and returns the payload of the camera's reply.
    pub fn command(&self, command: u16, payload: &[u8]) -> Result<Vec<u8>> {
        let mut next_tag = self.tag.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        let tag = *next_tag;
        *next_tag = tag.wrapping_add(1);
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

    pub fn read_register(&self, register: u16) -> Result<u16> {
        let reply = self.command(camera::READ_REGISTER, &camera::read_register_payload(register))?;
        camera::parse_read_register(&reply)
    }

    pub fn write_register(&self, register: u16, value: u16) -> Result<()> {
        let reply = self.command(camera::WRITE_REGISTER, &camera::write_register_payload(register, value))?;
        camera::check_write_register(&reply)
    }

    /// Starts the depth stream: 640×480 raw 11-bit values at 30 fps. It stops when dropped.
    pub fn start_depth(&self) -> Result<DepthStream<'_>> {
        let spec = StreamSpec {
            endpoint: depth::ENDPOINT,
            kind: StreamKind::Depth,
            frame_len: depth::PACKED_FRAME_LEN,
            start: &depth::START,
            stop: &depth::STOP,
            name: "a depth frame",
        };
        RawStream::start(self, spec).map(DepthStream)
    }

    /// Starts the color stream: 640×480 RGB at 30 fps. It stops when dropped.
    /// Cannot run at the same time as the IR stream.
    pub fn start_color(&self) -> Result<ColorStream<'_>> {
        let spec = StreamSpec {
            endpoint: video::ENDPOINT,
            kind: StreamKind::Video,
            frame_len: video::BAYER_FRAME_LEN,
            start: &video::COLOR_START,
            stop: &video::STOP,
            name: "a color frame",
        };
        RawStream::start(self, spec).map(ColorStream)
    }

    /// Starts the IR stream: 640×488 10-bit intensities at 30 fps. It stops when dropped.
    /// Cannot run at the same time as the color stream.
    pub fn start_ir(&self) -> Result<IrStream<'_>> {
        let spec = StreamSpec {
            endpoint: video::ENDPOINT,
            kind: StreamKind::Video,
            frame_len: video::IR_PACKED_FRAME_LEN,
            start: &video::IR_START,
            stop: &video::STOP,
            name: "an IR frame",
        };
        RawStream::start(self, spec).map(IrStream)
    }
}

struct StreamSpec {
    endpoint: u8,
    kind: StreamKind,
    frame_len: usize,
    start: &'static [(u16, u16)],
    stop: &'static [(u16, u16)],
    /// What a frame timeout says it was waiting for.
    name: &'static str,
}

/// A running stream of frames in the camera's wire format.
struct RawStream<'a> {
    camera: &'a Camera,
    reader: IsochReader,
    assembler: FrameAssembler,
    ready: VecDeque<RawFrame>,
    spec: StreamSpec,
}

impl<'a> RawStream<'a> {
    fn start(camera: &'a Camera, spec: StreamSpec) -> Result<Self> {
        let endpoint = camera
            .device
            .configuration()?
            .settings
            .iter()
            .flat_map(|s| &s.endpoints)
            .find(|e| e.address == spec.endpoint)
            .cloned()
            .ok_or_else(|| Error::Protocol(format!("camera has no endpoint {:#04x}", spec.endpoint)))?;
        // Queue transfers before starting the camera so that no data is missed; they fail
        // harmlessly until it streams.
        let reader = IsochReader::new(
            &camera.device,
            spec.endpoint,
            endpoint.bytes_per_interval() as usize,
            TRANSFERS,
            PACKETS_PER_TRANSFER,
        )?;
        let stream = Self {
            camera,
            reader,
            assembler: FrameAssembler::new(spec.kind, spec.frame_len),
            ready: VecDeque::new(),
            spec,
        };
        for &(register, value) in stream.spec.start {
            stream.camera.write_register(register, value)?;
        }
        Ok(stream)
    }

    fn next_frame(&mut self) -> Result<RawFrame> {
        let start = Instant::now();
        loop {
            if let Some(raw) = self.ready.pop_front() {
                return Ok(raw);
            }
            if start.elapsed() > FRAME_TIMEOUT {
                return Err(Error::Timeout(self.spec.name));
            }
            let (assembler, ready) = (&mut self.assembler, &mut self.ready);
            self.reader.read(FRAME_TIMEOUT, |packet| ready.extend(assembler.push(packet)))?;
        }
    }
}

impl Drop for RawStream<'_> {
    fn drop(&mut self) {
        for &(register, value) in self.spec.stop {
            let _ = self.camera.write_register(register, value);
        }
    }
}

pub struct DepthStream<'a>(RawStream<'a>);

impl DepthStream<'_> {
    /// Waits for the next complete frame.
    pub fn next_frame(&mut self) -> Result<DepthFrame> {
        self.0.next_frame().map(|raw| DepthFrame::unpack(&raw))
    }

    /// Frames lost so far to dropped or out-of-order packets.
    pub fn dropped_frames(&self) -> u64 {
        self.0.assembler.dropped_frames()
    }
}

pub struct ColorStream<'a>(RawStream<'a>);

impl ColorStream<'_> {
    /// Waits for the next complete frame.
    pub fn next_frame(&mut self) -> Result<ColorFrame> {
        self.0.next_frame().map(|raw| ColorFrame::from_bayer(&raw))
    }

    /// Frames lost so far to dropped or out-of-order packets.
    pub fn dropped_frames(&self) -> u64 {
        self.0.assembler.dropped_frames()
    }
}

pub struct IrStream<'a>(RawStream<'a>);

impl IrStream<'_> {
    /// Waits for the next complete frame.
    pub fn next_frame(&mut self) -> Result<IrFrame> {
        self.0.next_frame().map(|raw| IrFrame::unpack(&raw))
    }

    /// Frames lost so far to dropped or out-of-order packets.
    pub fn dropped_frames(&self) -> u64 {
        self.0.assembler.dropped_frames()
    }
}
