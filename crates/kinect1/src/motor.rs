//! Tilt motor, LED and accelerometer: types and wire protocol, with no I/O.
//!
//! Original Xbox 360 Kinects (model 1414) have a separate motor device (PID 02B0). Kinect for
//! Windows and model 1473 units instead take motor commands over bulk endpoints of the audio
//! device's first interface, once its firmware is running; [`audio`] encodes that protocol.

use crate::{Error, Result};

/// Accelerometer counts per g (Kionix KXSD9 datasheet).
pub const COUNTS_PER_G: f64 = 819.0;
const STANDARD_GRAVITY: f64 = 9.80665;

/// Tilt limits Microsoft's SDK enforces; the mechanism itself goes a little further.
pub const MIN_TILT_DEGREES: i8 = -27;
pub const MAX_TILT_DEGREES: i8 = 27;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Led {
    Off,
    Green,
    Red,
    /// Not available on Kinect for Windows / model 1473 units.
    Yellow,
    BlinkGreen,
    /// Not available on Kinect for Windows / model 1473 units.
    BlinkRedYellow,
}

/// Raw accelerometer reading, in counts (see [`COUNTS_PER_G`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Accelerometer {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Accelerometer {
    /// Acceleration in m/s² along x, y and z.
    pub fn meters_per_second_squared(self) -> [f64; 3] {
        [self.x, self.y, self.z].map(|c| f64::from(c) / COUNTS_PER_G * STANDARD_GRAVITY)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TiltStatus {
    Stopped,
    /// Stopped at the end of its travel.
    Limit,
    Moving,
    /// The last move gave up without the motor turning. Kinect for Windows and model 1473 units
    /// report this when the camera has no working driver: the motor only runs while it does.
    Failed,
    /// A code without a known meaning.
    Other(u32),
}

impl TiltStatus {
    pub fn from_code(code: u32) -> Self {
        match code {
            0 => Self::Stopped,
            1 => Self::Limit,
            4 => Self::Moving,
            8 => Self::Failed,
            other => Self::Other(other),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TiltState {
    pub accelerometer: Accelerometer,
    /// Tilt angle in degrees.
    pub angle: i8,
    pub status: TiltStatus,
    /// A tilt command is being carried out.
    pub busy: bool,
}

/// Checks that `degrees` is a tilt the motor may be asked for.
pub fn check_tilt(degrees: i8) -> Result<()> {
    if (MIN_TILT_DEGREES..=MAX_TILT_DEGREES).contains(&degrees) {
        Ok(())
    } else {
        Err(Error::InvalidArgument(format!(
            "tilt of {degrees}° is outside {MIN_TILT_DEGREES}..={MAX_TILT_DEGREES}°"
        )))
    }
}

/// Motor protocol spoken through the audio device (Kinect for Windows, model 1473).
///
/// Every command is little-endian `u32`s `[magic, tag, arg1, command, (arg2)]` written to
/// [`audio::OUT_ENDPOINT`]. The device answers each command with a 12-byte acknowledgement on
/// [`audio::IN_ENDPOINT`]; a state request gets its data reply first.
pub mod audio {
    use super::*;

    pub const OUT_ENDPOINT: u8 = 0x01;
    pub const IN_ENDPOINT: u8 = 0x81;

    const COMMAND_MAGIC: u32 = 0x0602_2009;
    const ACK_MAGIC: u32 = 0x0a6f_e000;
    pub const ACK_LEN: usize = 12;

    const READ_STATE: u32 = 0x8032;
    const SET_TILT: u32 = 0x803b;
    const SET_LED: u32 = 0x10;
    /// Bytes of state data that follow the 16-byte reply header.
    pub const STATE_LEN: usize = 104;

    fn command(tag: u32, arg1: u32, command: u32, arg2: Option<u32>) -> Vec<u8> {
        [COMMAND_MAGIC, tag, arg1, command].into_iter().chain(arg2).flat_map(u32::to_le_bytes).collect()
    }

    fn u32_at(bytes: &[u8], at: usize) -> u32 {
        u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
    }

    pub fn read_state(tag: u32) -> Vec<u8> {
        command(tag, STATE_LEN as u32, READ_STATE, None)
    }

    pub fn set_tilt(tag: u32, degrees: i8) -> Result<Vec<u8>> {
        check_tilt(degrees)?;
        Ok(command(tag, 0, SET_TILT, Some(i32::from(degrees) as u32)))
    }

    pub fn set_led(tag: u32, led: Led) -> Result<Vec<u8>> {
        let code = match led {
            Led::Off => 1,
            Led::BlinkGreen => 2,
            Led::Green => 3,
            Led::Red => 4,
            Led::Yellow | Led::BlinkRedYellow => {
                return Err(Error::Unsupported("this LED color on Kinect for Windows / model 1473"));
            }
        };
        Ok(command(tag, 0, SET_LED, Some(code)))
    }

    /// Checks the acknowledgement that ends every command.
    pub fn check_ack(bytes: &[u8]) -> Result<()> {
        if bytes.len() != ACK_LEN || u32_at(bytes, 0) != ACK_MAGIC {
            return Err(Error::Protocol(format!("unexpected motor acknowledgement {bytes:02x?}")));
        }
        match u32_at(bytes, 8) {
            0 => Ok(()),
            status => Err(Error::Protocol(format!("motor command failed with status {status}"))),
        }
    }

    /// Parses the data reply to [`read_state`]: a 16-byte header whose second word has a busy bit,
    /// then `i32` accelerometer x, y, z, the tilt angle in degrees and the motor status. Further
    /// fields are not understood.
    pub fn parse_state(bytes: &[u8]) -> Result<TiltState> {
        if bytes.len() < 36 {
            return Err(Error::Protocol(format!("motor state reply is {} bytes", bytes.len())));
        }
        let field = |i: usize| u32_at(bytes, 16 + 4 * i) as i32;
        Ok(TiltState {
            accelerometer: Accelerometer { x: field(0) as i16, y: field(1) as i16, z: field(2) as i16 },
            angle: field(3) as i8,
            status: TiltStatus::from_code(field(4) as u32),
            busy: u32_at(bytes, 4) & 1 != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_audio_commands() {
        assert_eq!(
            audio::read_state(7),
            [0x09, 0x20, 0x02, 0x06, 7, 0, 0, 0, 104, 0, 0, 0, 0x32, 0x80, 0, 0]
        );
        let tilt = audio::set_tilt(1, -5).unwrap();
        assert_eq!(tilt.len(), 20);
        assert_eq!(&tilt[12..], [0x3b, 0x80, 0, 0, 0xfb, 0xff, 0xff, 0xff]);
        assert_eq!(&audio::set_led(2, Led::Red).unwrap()[12..], [0x10, 0, 0, 0, 4, 0, 0, 0]);
    }

    #[test]
    fn rejects_unsafe_tilt_and_unsupported_leds() {
        assert!(audio::set_tilt(0, 28).is_err());
        assert!(audio::set_tilt(0, -27).is_ok());
        assert!(matches!(audio::set_led(0, Led::Yellow), Err(Error::Unsupported(_))));
    }

    #[test]
    fn checks_acknowledgements() {
        let ok = [0x00, 0xe0, 0x6f, 0x0a, 3, 0, 0, 0, 0, 0, 0, 0];
        assert!(audio::check_ack(&ok).is_ok());
        let mut failed = ok;
        failed[8] = 1;
        assert!(audio::check_ack(&failed).is_err());
        assert!(audio::check_ack(&ok[..8]).is_err());
    }

    #[test]
    fn parses_state_reply() {
        let mut reply = vec![0u8; audio::STATE_LEN];
        for (i, v) in [-12i32, 819, -40, -3, 4].into_iter().enumerate() {
            reply[16 + 4 * i..20 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        reply[4] = 1;
        let state = audio::parse_state(&reply).unwrap();
        assert!(state.busy);
        assert_eq!(state.accelerometer, Accelerometer { x: -12, y: 819, z: -40 });
        assert_eq!(state.angle, -3);
        assert_eq!(state.status, TiltStatus::Moving);
        assert_eq!(TiltStatus::from_code(8), TiltStatus::Failed);
        assert!((state.accelerometer.meters_per_second_squared()[1] - STANDARD_GRAVITY).abs() < 1e-9);
    }
}
