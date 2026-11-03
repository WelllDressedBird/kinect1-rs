//! USB identities of the parts of a Kinect v1.

pub const MICROSOFT_VID: u16 = 0x045e;

/// One of the USB devices behind the Kinect's internal hub.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Component {
    /// Depth, color and IR cameras.
    Camera,
    /// Tilt motor, LED and accelerometer.
    Motor,
    /// Microphone array (and, on some units, motor and LED control).
    Audio,
    /// The Kinect's internal USB hub.
    Hub,
}

impl Component {
    pub fn from_product_id(pid: u16) -> Option<Self> {
        match pid {
            0x02ae => Some(Self::Camera),
            0x02b0 => Some(Self::Motor),
            // 0x02ad before its firmware is loaded; 0x02bb once the Microsoft driver has loaded it.
            0x02ad | 0x02bb => Some(Self::Audio),
            0x02c2 => Some(Self::Hub),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Camera => "camera",
            Self::Motor => "motor",
            Self::Audio => "audio",
            Self::Hub => "hub",
        }
    }
}

/// What a Windows device instance id (e.g. `USB\VID_045E&PID_02BB&MI_01\6&1234`) says about a device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UsbIdentity {
    pub product_id: u16,
    pub component: Component,
    /// Interface number for one function of a composite device (`MI_xx`).
    pub interface: Option<u8>,
}

/// Recognises Kinect devices from their instance id. Returns `None` for anything else.
pub fn identify(instance_id: &str) -> Option<UsbIdentity> {
    let id = instance_id.to_ascii_uppercase();
    let hardware = id.strip_prefix("USB\\")?.split('\\').next()?;
    let mut vid = None;
    let mut pid = None;
    let mut interface = None;
    for part in hardware.split('&') {
        if let Some(hex) = part.strip_prefix("VID_") {
            vid = u16::from_str_radix(hex, 16).ok();
        } else if let Some(hex) = part.strip_prefix("PID_") {
            pid = u16::from_str_radix(hex, 16).ok();
        } else if let Some(hex) = part.strip_prefix("MI_") {
            interface = u8::from_str_radix(hex, 16).ok();
        }
    }
    if vid? != MICROSOFT_VID {
        return None;
    }
    let product_id = pid?;
    Some(UsbIdentity { product_id, component: Component::from_product_id(product_id)?, interface })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifies_kinect_parts() {
        let camera = identify(r"USB\VID_045E&PID_02AE\0000000000000000").unwrap();
        assert_eq!(camera, UsbIdentity { product_id: 0x02ae, component: Component::Camera, interface: None });

        let audio_fn = identify(r"usb\vid_045e&pid_02bb&mi_01\a&b17302b&1&0001").unwrap();
        assert_eq!((audio_fn.component, audio_fn.interface), (Component::Audio, Some(1)));
    }

    #[test]
    fn ignores_other_devices() {
        assert!(identify(r"USB\VID_045E&PID_0745\5&1").is_none()); // a Microsoft mouse receiver
        assert!(identify(r"USB\VID_1234&PID_02AE\x").is_none());
        assert!(identify(r"HID\VID_045E&PID_02AE\x").is_none());
    }
}
