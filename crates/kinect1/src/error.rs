pub type Result<T, E = Error> = std::result::Result<T, E>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no Kinect {0} is connected")]
    NotFound(&'static str),

    #[error(
        "the Kinect {component} uses the \"{driver}\" driver; the WinUSB backend needs WinUSB. \
         Install it for that device with Zadig (see the README)"
    )]
    WrongDriver { component: &'static str, driver: String },

    #[error("Kinect protocol error: {0}")]
    Protocol(String),

    #[error(
        "the Kinect audio firmware is not running (the audio device shows as 045E 02AD), so the motor \
         can't be controlled. The Kinect for Windows runtime loads it: start the KinectManagement \
         service, or replug the Kinect while it runs"
    )]
    AudioFirmwareNotLoaded,

    #[error("timed out waiting for {0}")]
    Timeout(&'static str),

    #[error("invalid argument: {0}")]
    InvalidArgument(String),

    #[error("not supported: {0}")]
    Unsupported(&'static str),

    #[cfg(windows)]
    #[error(transparent)]
    Os(#[from] windows_core::Error),
}
