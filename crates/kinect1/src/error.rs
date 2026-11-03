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

    #[error("invalid argument: {0}")]
    InvalidArgument(String),

    #[error("not supported: {0}")]
    Unsupported(&'static str),

    #[cfg(windows)]
    #[error(transparent)]
    Os(#[from] windows_core::Error),
}
