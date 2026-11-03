//! Parsing of standard USB descriptors (USB 2.0 specification, chapter 9).

const DEVICE: u8 = 1;
const CONFIGURATION: u8 = 2;
const INTERFACE: u8 = 4;
const ENDPOINT: u8 = 5;

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([bytes[at], bytes[at + 1]])
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceDescriptor {
    /// BCD, e.g. 0x0200 for USB 2.0.
    pub usb_version: u16,
    pub class: u8,
    pub max_packet_size0: u8,
    pub vendor_id: u16,
    pub product_id: u16,
    /// BCD device release number.
    pub device_version: u16,
    pub manufacturer_index: u8,
    pub product_index: u8,
    pub serial_index: u8,
    pub num_configurations: u8,
}

impl DeviceDescriptor {
    pub const LEN: usize = 18;

    pub fn parse(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::LEN || bytes[0] as usize != Self::LEN || bytes[1] != DEVICE {
            return None;
        }
        Some(Self {
            usb_version: u16_at(bytes, 2),
            class: bytes[4],
            max_packet_size0: bytes[7],
            vendor_id: u16_at(bytes, 8),
            product_id: u16_at(bytes, 10),
            device_version: u16_at(bytes, 12),
            manufacturer_index: bytes[14],
            product_index: bytes[15],
            serial_index: bytes[16],
            num_configurations: bytes[17],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferType {
    Control,
    Isochronous,
    Bulk,
    Interrupt,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Endpoint {
    /// Bit 7 set = IN (device to host).
    pub address: u8,
    pub transfer_type: TransferType,
    /// Bytes per transaction (bits 0–10 of wMaxPacketSize).
    pub max_packet_size: u16,
    /// High-speed high-bandwidth endpoints do up to 3 transactions per microframe.
    pub transactions_per_microframe: u8,
    pub interval: u8,
}

impl Endpoint {
    pub fn is_in(&self) -> bool {
        self.address & 0x80 != 0
    }

    /// Bytes the endpoint can move per service interval.
    pub fn bytes_per_interval(&self) -> u32 {
        u32::from(self.max_packet_size) * u32::from(self.transactions_per_microframe)
    }
}

/// One alternate setting of an interface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InterfaceSetting {
    pub number: u8,
    pub alternate: u8,
    pub class: u8,
    pub subclass: u8,
    pub protocol: u8,
    pub endpoints: Vec<Endpoint>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Configuration {
    pub value: u8,
    pub max_power_ma: u16,
    pub settings: Vec<InterfaceSetting>,
}

impl Configuration {
    pub const HEADER_LEN: usize = 9;

    /// Total length of the configuration and everything under it, read from its 9-byte header.
    pub fn total_length(header: &[u8]) -> Option<u16> {
        (header.len() >= Self::HEADER_LEN && header[1] == CONFIGURATION).then(|| u16_at(header, 2))
    }

    /// Parses a full configuration descriptor set. Returns `None` if it is malformed.
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::HEADER_LEN || bytes[1] != CONFIGURATION {
            return None;
        }
        let mut config = Self { value: bytes[5], max_power_ma: u16::from(bytes[8]) * 2, settings: Vec::new() };

        let mut at = 0;
        while at < bytes.len() {
            let len = bytes[at] as usize;
            if len < 2 || at + len > bytes.len() {
                return None;
            }
            let d = &bytes[at..at + len];
            match d[1] {
                INTERFACE if len >= 9 => config.settings.push(InterfaceSetting {
                    number: d[2],
                    alternate: d[3],
                    class: d[5],
                    subclass: d[6],
                    protocol: d[7],
                    endpoints: Vec::new(),
                }),
                ENDPOINT if len >= 7 => {
                    let raw = u16_at(d, 4);
                    let endpoint = Endpoint {
                        address: d[2],
                        transfer_type: match d[3] & 0b11 {
                            0 => TransferType::Control,
                            1 => TransferType::Isochronous,
                            2 => TransferType::Bulk,
                            _ => TransferType::Interrupt,
                        },
                        max_packet_size: raw & 0x7ff,
                        transactions_per_microframe: ((raw >> 11) & 0b11) as u8 + 1,
                        interval: d[6],
                    };
                    config.settings.last_mut()?.endpoints.push(endpoint);
                }
                _ => {}
            }
            at += len;
        }
        Some(config)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_device_descriptor() {
        let bytes = [18, 1, 0x00, 0x02, 0, 0, 0, 64, 0x5e, 0x04, 0xae, 0x02, 0x05, 0x01, 1, 2, 3, 1];
        let d = DeviceDescriptor::parse(&bytes).unwrap();
        assert_eq!((d.vendor_id, d.product_id), (0x045e, 0x02ae));
        assert_eq!(d.usb_version, 0x0200);
        assert_eq!(d.serial_index, 3);
        assert!(DeviceDescriptor::parse(&bytes[..17]).is_none());
    }

    #[test]
    fn parses_alternate_settings_and_high_bandwidth_iso_endpoints() {
        #[rustfmt::skip]
        let bytes = [
            9, 2, 41, 0, 1, 1, 0, 0x80, 250,   // configuration, 41 bytes total, 500 mA
            9, 4, 0, 0, 0, 0xff, 0xff, 0xff, 0, // interface 0 alt 0, no endpoints
            9, 4, 0, 1, 2, 0xff, 0xff, 0xff, 0, // interface 0 alt 1, two endpoints
            7, 5, 0x81, 0x01, 0xc0, 0x13, 1,    // iso IN 0x81, 960 bytes x 3
            7, 5, 0x02, 0x02, 0x00, 0x02, 0,    // bulk OUT 0x02, 512 bytes
        ];
        assert_eq!(Configuration::total_length(&bytes[..9]), Some(41));
        let config = Configuration::parse(&bytes).unwrap();
        assert_eq!(config.max_power_ma, 500);
        assert_eq!(config.settings.len(), 2);
        let alt1 = &config.settings[1];
        assert_eq!((alt1.number, alt1.alternate), (0, 1));
        let iso = &alt1.endpoints[0];
        assert!(iso.is_in());
        assert_eq!(iso.transfer_type, TransferType::Isochronous);
        assert_eq!((iso.max_packet_size, iso.transactions_per_microframe), (960, 3));
        assert_eq!(iso.bytes_per_interval(), 2880);
        assert!(!alt1.endpoints[1].is_in());
    }

    #[test]
    fn rejects_truncated_descriptors() {
        let bytes = [9, 2, 18, 0, 1, 1, 0, 0x80, 50, 9, 4, 0, 0];
        assert!(Configuration::parse(&bytes).is_none());
    }
}
