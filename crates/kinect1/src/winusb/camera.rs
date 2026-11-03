//! Commands, registers and streams of the depth/color camera.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use super::isoch::IsochReader;
use super::{UsbNode, WinUsbDevice, scan};
use crate::depth::{self, Calibration, DepthFrame};
use crate::registration::{self, RegInfo, Registration};
use crate::stream::{FrameAssembler, RawFrame, StreamKind};
use crate::video::{self, ColorFrame, IrFrame, Resolution};
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

    /// Reads the camera's depth calibration, for converting depth to millimetres.
    pub fn depth_calibration(&self) -> Result<Calibration> {
        let fixed = self.command(camera::GET_FIXED_PARAMS, &depth::FIXED_PARAMS_PAYLOAD)?;
        let shift = self.command(camera::ALGORITHM_PARAMS, &depth::CONST_SHIFT_PAYLOAD)?;
        Calibration::parse(&fixed, &shift)
    }

    /// Reads the camera's model for aligning depth with the 640×480 color image.
    pub fn registration(&self) -> Result<Registration> {
        let calibration = self.depth_calibration()?;
        let reg_info = RegInfo::parse(&self.command(camera::ALGORITHM_PARAMS, &registration::REG_INFO_PAYLOAD)?)?;
        let pad = registration::parse_pad_start_lines(
            &self.command(camera::ALGORITHM_PARAMS, &registration::PAD_INFO_PAYLOAD)?,
        )?;
        Ok(Registration::new(&reg_info, pad, &calibration))
    }

    /// Starts the depth stream: 640×480 raw 11-bit values at 30 fps. It stops when dropped.
    pub fn start_depth(&self) -> Result<DepthStream<'_>> {
        let spec = StreamSpec {
            endpoint: depth::ENDPOINT,
            kind: StreamKind::Depth,
            frame_len: depth::PACKED_FRAME_LEN,
            variable: false,
            start: &depth::START,
            stop: &depth::STOP,
            name: "a depth frame",
        };
        RawStream::start(self, spec).map(|raw| DepthStream(raw, ()))
    }

    /// Starts the color stream: 640×480 RGB at 30 fps. It stops when dropped.
    /// Cannot run at the same time as the IR stream.
    pub fn start_color(&self) -> Result<ColorStream<'_>> {
        self.start_color_at(Resolution::Medium)
    }

    /// Starts the color stream at `resolution`. It stops when dropped.
    /// Cannot run at the same time as the IR stream.
    pub fn start_color_at(&self, resolution: Resolution) -> Result<ColorStream<'_>> {
        let spec = StreamSpec {
            endpoint: video::ENDPOINT,
            kind: StreamKind::Video,
            frame_len: resolution.bayer_frame_len(),
            variable: resolution == Resolution::High,
            start: resolution.color_start(),
            stop: &video::STOP,
            name: "a color frame",
        };
        RawStream::start(self, spec).map(|raw| ColorStream(raw, resolution))
    }

    /// Starts the IR stream: 640×488 10-bit intensities at 30 fps. It stops when dropped.
    /// Cannot run at the same time as the color stream.
    pub fn start_ir(&self) -> Result<IrStream<'_>> {
        self.start_ir_at(Resolution::Medium)
    }

    /// Starts the IR stream at `resolution`. It stops when dropped. Cannot run at the same time
    /// as the color stream, nor at [`Resolution::High`] alongside depth.
    pub fn start_ir_at(&self, resolution: Resolution) -> Result<IrStream<'_>> {
        let spec = StreamSpec {
            endpoint: video::ENDPOINT,
            kind: StreamKind::Video,
            frame_len: resolution.ir_packed_frame_len(),
            variable: false,
            start: resolution.ir_start(),
            stop: &video::STOP,
            name: "an IR frame",
        };
        RawStream::start(self, spec).map(|raw| IrStream(raw, resolution))
    }
}

struct StreamSpec {
    endpoint: u8,
    kind: StreamKind,
    frame_len: usize,
    /// Frames may end before `frame_len`; see [`FrameAssembler::variable`].
    variable: bool,
    start: &'static [(u16, u16)],
    stop: &'static [(u16, u16)],
    /// What a frame timeout says it was waiting for.
    name: &'static str,
}

/// Frames the reader thread holds for the caller; older ones are skipped when it falls behind.
const QUEUED_FRAMES: usize = 2;
/// How often the reader thread checks whether the stream is being stopped.
const READ_POLL: Duration = Duration::from_millis(100);

/// What the reader thread shares with the stream's owner.
struct Shared {
    state: Mutex<SharedState>,
    frame_ready: Condvar,
    stop: AtomicBool,
}

#[derive(Default)]
struct SharedState {
    frames: VecDeque<RawFrame>,
    /// Set when the reader thread has ended; holds its error until the caller has seen it.
    ended: Option<Option<Error>>,
    dropped: u64,
    skipped: u64,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, SharedState> {
        self.state.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
    }
}

/// Reads packets on its own thread so the USB transfers keep flowing however slowly frames are
/// taken, and hands completed frames over through `shared`.
fn read_frames(mut reader: IsochReader, mut assembler: FrameAssembler, shared: &Shared) {
    let mut completed = Vec::new();
    let error = loop {
        if shared.stop.load(Ordering::Relaxed) {
            break None;
        }
        match reader.read(READ_POLL, |packet| completed.extend(assembler.push(packet))) {
            Ok(()) | Err(Error::Timeout(_)) => {}
            Err(e) => break Some(e),
        }
        if completed.is_empty() {
            continue;
        }
        let mut state = shared.lock();
        for frame in completed.drain(..) {
            if state.frames.len() == QUEUED_FRAMES {
                state.frames.pop_front();
                state.skipped += 1;
            }
            state.frames.push_back(frame);
        }
        state.dropped = assembler.dropped_frames();
        drop(state);
        shared.frame_ready.notify_all();
    };
    shared.lock().ended = Some(error);
    shared.frame_ready.notify_all();
}

/// A running stream of frames in the camera's wire format.
struct RawStream<'a> {
    camera: &'a Camera,
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
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
        let assembler = match spec.variable {
            true => FrameAssembler::variable(spec.kind, spec.frame_len),
            false => FrameAssembler::new(spec.kind, spec.frame_len),
        };
        let shared = Arc::new(Shared {
            state: Mutex::default(),
            frame_ready: Condvar::new(),
            stop: AtomicBool::new(false),
        });
        let thread = std::thread::Builder::new()
            .name(format!("kinect1 {:#04x}", spec.endpoint))
            .spawn({
                let shared = Arc::clone(&shared);
                move || read_frames(reader, assembler, &shared)
            })
            .map_err(|e| Error::Protocol(format!("could not start the stream thread: {e}")))?;
        // From here on, Drop stops the camera and the thread.
        let stream = Self { camera, shared, thread: Some(thread), spec };
        for &(register, value) in stream.spec.start {
            stream.camera.write_register(register, value)?;
        }
        Ok(stream)
    }

    fn next_frame(&mut self) -> Result<RawFrame> {
        let deadline = Instant::now() + FRAME_TIMEOUT;
        let mut state = self.shared.lock();
        loop {
            if let Some(frame) = state.frames.pop_front() {
                return Ok(frame);
            }
            if let Some(error) = &mut state.ended {
                return Err(error.take().unwrap_or(Error::Protocol(format!("{} stream has stopped", self.spec.name))));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(Error::Timeout(self.spec.name));
            }
            state = self.shared.frame_ready.wait_timeout(state, left).unwrap_or_else(|p| p.into_inner()).0;
        }
    }

    fn dropped_frames(&self) -> u64 {
        self.shared.lock().dropped
    }

    fn skipped_frames(&self) -> u64 {
        self.shared.lock().skipped
    }
}

impl Drop for RawStream<'_> {
    fn drop(&mut self) {
        for &(register, value) in self.spec.stop {
            let _ = self.camera.write_register(register, value);
        }
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

macro_rules! stream_type {
    ($(#[$doc:meta])* $name:ident, $frame:ty, $format:ty, $convert:expr) => {
        $(#[$doc])*
        pub struct $name<'a>(RawStream<'a>, $format);

        impl $name<'_> {
            /// Waits for the next complete frame. Returns the oldest frame not yet taken; if the
            /// caller falls behind, older frames are skipped rather than queued.
            pub fn next_frame(&mut self) -> Result<$frame> {
                let raw = self.0.next_frame()?;
                Ok($convert(&raw, self.1))
            }

            /// Frames lost so far to dropped or out-of-order packets.
            pub fn dropped_frames(&self) -> u64 {
                self.0.dropped_frames()
            }

            /// Frames received intact but discarded because the caller did not take them in time.
            pub fn skipped_frames(&self) -> u64 {
                self.0.skipped_frames()
            }
        }
    };
}

stream_type!(
    /// A running depth stream, read on its own thread. Stops when dropped.
    DepthStream,
    DepthFrame,
    (),
    |raw, ()| DepthFrame::unpack(raw)
);
stream_type!(
    /// A running color stream, read on its own thread. Stops when dropped.
    ColorStream,
    ColorFrame,
    Resolution,
    ColorFrame::from_bayer
);
stream_type!(
    /// A running IR stream, read on its own thread. Stops when dropped.
    IrStream,
    IrFrame,
    Resolution,
    IrFrame::unpack
);
