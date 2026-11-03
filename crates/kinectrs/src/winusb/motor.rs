//! Tilt motor, LED and accelerometer, through the motor device (model 1414) or the audio device
//! (Kinect for Windows, model 1473).

use std::sync::Mutex;
use std::time::{Duration, Instant};

use super::{UsbNode, WinUsbDevice, scan};
use crate::motor::{Led, TiltState, TiltStatus, audio, classic};
use crate::{Component, Error, Result};

const TIMEOUT: Duration = Duration::from_millis(250);
/// How often [`Motor::tilt`] polls, how long it waits for the move to start, and to finish.
const TILT_POLL: Duration = Duration::from_millis(100);
const TILT_START: Duration = Duration::from_secs(1);
const TILT_FINISH: Duration = Duration::from_secs(10);

const VENDOR_OUT: u8 = 0x40;
const VENDOR_IN: u8 = 0xc0;

/// Which device and protocol drive the motor.
enum Link {
    /// Model 1414's motor device, with control transfers.
    Classic,
    /// The audio device's bulk endpoints. Holds the tag of the next command, held for a whole
    /// command so command/reply pairs don't interleave.
    Audio(Mutex<u32>),
}

/// The tilt motor, LED and accelerometer. Commands may be sent from several threads.
pub struct Motor {
    device: WinUsbDevice,
    link: Link,
}

impl Motor {
    /// Opens the motor of the first connected Kinect: its motor device on a model 1414, otherwise
    /// the audio device's first interface.
    pub fn open() -> Result<Self> {
        let nodes = scan()?;
        if let Some(motor) = nodes.iter().find(|n| n.component == Component::Motor) {
            return Self::open_node(motor);
        }
        let mut candidates = nodes.iter().filter(|n| n.component == Component::Audio && n.interface.unwrap_or(0) == 0);
        let first = candidates.clone().next().ok_or(Error::NotFound("motor or audio device"))?;
        Self::open_node(candidates.find(|n| n.is_winusb()).unwrap_or(first))
    }

    /// Opens the motor through `node`: a model 1414 motor device, or the first interface of an
    /// audio device.
    pub fn open_node(node: &UsbNode) -> Result<Self> {
        if node.component == Component::Motor {
            return Ok(Self { device: WinUsbDevice::open(node)?, link: Link::Classic });
        }
        if Component::is_audio_without_firmware(node.product_id) {
            return Err(Error::AudioFirmwareNotLoaded);
        }
        let device = WinUsbDevice::open(node)?;
        for endpoint in [audio::OUT_ENDPOINT, audio::IN_ENDPOINT] {
            device.set_timeout(endpoint, TIMEOUT)?;
        }
        device.flush(audio::IN_ENDPOINT)?;
        Ok(Self { device, link: Link::Audio(Mutex::new(0)) })
    }

    /// Sends an audio-device command and returns the data reply that precedes the
    /// acknowledgement, if any.
    fn send(&self, tag: &Mutex<u32>, command: impl FnOnce(u32) -> Result<Vec<u8>>) -> Result<Vec<u8>> {
        let mut tag = tag.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        let bytes = command(*tag)?;
        *tag = tag.wrapping_add(1);
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

    pub fn state(&self) -> Result<TiltState> {
        match &self.link {
            Link::Classic => {
                let mut buf = [0u8; classic::STATE_LEN];
                let len = self.device.control_in(VENDOR_IN, classic::READ_STATE, 0, 0, &mut buf)?;
                classic::parse_state(&buf[..len])
            }
            Link::Audio(tag) => audio::parse_state(&self.send(tag, |tag| Ok(audio::read_state(tag)))?),
        }
    }

    /// Starts tilting to `degrees` from level. Returns before the motor has finished moving; see
    /// [`Motor::tilt`] to wait.
    pub fn set_tilt(&self, degrees: i8) -> Result<()> {
        match &self.link {
            Link::Classic => {
                let value = classic::tilt_value(degrees)?;
                self.device.control_out(VENDOR_OUT, classic::SET_TILT, value, 0, &[]).map(drop)
            }
            Link::Audio(tag) => self.send(tag, |tag| audio::set_tilt(tag, degrees)).map(drop),
        }
    }

    /// Tilts to `degrees` from level and waits until the motor has stopped, as Microsoft's SDK
    /// does. Fails with [`Error::TiltFailed`] if the motor gave up without moving.
    ///
    /// Microsoft advises at most one tilt a second and 15 in any 20 seconds, to spare the motor.
    pub fn tilt(&self, degrees: i8) -> Result<TiltState> {
        self.set_tilt(degrees)?;
        let start = Instant::now();
        let mut started = false;
        loop {
            std::thread::sleep(TILT_POLL);
            let state = self.state()?;
            started |= state.busy;
            // Tiny moves can finish before the busy bit is ever seen.
            if (started && !state.busy) || (!started && start.elapsed() > TILT_START) {
                return match state.status {
                    TiltStatus::Failed => Err(Error::TiltFailed),
                    _ => Ok(state),
                };
            }
            if start.elapsed() > TILT_FINISH {
                return Err(Error::Timeout("the tilt motor to stop"));
            }
        }
    }

    pub fn set_led(&self, led: Led) -> Result<()> {
        match &self.link {
            Link::Classic => {
                self.device.control_out(VENDOR_OUT, classic::SET_LED, classic::led_value(led), 0, &[]).map(drop)
            }
            Link::Audio(tag) => self.send(tag, |tag| audio::set_led(tag, led)).map(drop),
        }
    }
}
