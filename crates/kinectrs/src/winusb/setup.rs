//! Finding Kinect devices and their WinUSB device paths with SetupAPI.

use windows::Win32::Devices::DeviceAndDriverInstallation::*;
use windows::Win32::System::Registry::{HKEY, KEY_READ, REG_VALUE_TYPE, RegCloseKey, RegQueryValueExW};
use windows_core::{GUID, PCWSTR, w};

use crate::ids::{Component, identify};
use crate::{Error, Result};

/// A Kinect USB device (or one function of a composite device) as Windows sees it.
#[derive(Clone, Debug)]
pub struct UsbNode {
    pub instance_id: String,
    pub product_id: u16,
    pub component: Component,
    /// Interface number when this is one function of a composite device.
    pub interface: Option<u8>,
    pub description: String,
    /// Name of the driver service bound to the device, e.g. `WinUSB`.
    pub driver: Option<String>,
    /// Device Manager problem code (e.g. 39 = driver failed to load).
    pub problem: Option<u32>,
    /// Device interface classes registered for WinUSB access (set by the WinUSB INF, e.g. Zadig's).
    interface_guids: Vec<GUID>,
}

impl UsbNode {
    pub fn is_winusb(&self) -> bool {
        self.driver.as_deref().is_some_and(|d| d.eq_ignore_ascii_case("WinUSB"))
    }
}

/// Owns an `HDEVINFO` set.
struct DeviceSet(HDEVINFO);

impl Drop for DeviceSet {
    fn drop(&mut self) {
        let _ = unsafe { SetupDiDestroyDeviceInfoList(self.0) };
    }
}

fn utf16_until_nul(units: &[u16]) -> String {
    let end = units.iter().position(|&u| u == 0).unwrap_or(units.len());
    String::from_utf16_lossy(&units[..end])
}

fn bytes_to_utf16(bytes: &[u8]) -> Vec<u16> {
    bytes.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect()
}

fn instance_id(set: &DeviceSet, data: &SP_DEVINFO_DATA) -> Result<String> {
    let mut buf = [0u16; 512];
    unsafe { SetupDiGetDeviceInstanceIdW(set.0, data, Some(&mut buf), None)? };
    Ok(utf16_until_nul(&buf))
}

fn string_property(set: &DeviceSet, data: &SP_DEVINFO_DATA, property: SETUP_DI_REGISTRY_PROPERTY) -> Option<String> {
    let mut buf = [0u8; 1024];
    let mut size = 0u32;
    unsafe { SetupDiGetDeviceRegistryPropertyW(set.0, data, property, None, Some(&mut buf), Some(&mut size)).ok()? };
    let text = utf16_until_nul(&bytes_to_utf16(&buf[..size as usize]));
    (!text.is_empty()).then_some(text)
}

fn problem_code(data: &SP_DEVINFO_DATA) -> Option<u32> {
    let mut status = CM_DEVNODE_STATUS_FLAGS(0);
    let mut problem = CM_PROB(0);
    let ok = unsafe { CM_Get_DevNode_Status(&mut status, &mut problem, data.DevInst, 0) } == CR_SUCCESS;
    (ok && status.0 & DN_HAS_PROBLEM.0 != 0).then_some(problem.0)
}

/// Parses `{6B5F2D3A-8E41-4C7B-9A52-3F0D1E7C4B19}` (braces optional).
pub(crate) fn parse_guid(text: &str) -> Option<GUID> {
    let hex: String = text.trim().trim_start_matches('{').trim_end_matches('}').chars().filter(|&c| c != '-').collect();
    if hex.len() != 32 {
        return None;
    }
    u128::from_str_radix(&hex, 16).ok().map(GUID::from_u128)
}

fn query_registry_strings(key: HKEY, name: PCWSTR) -> Vec<String> {
    let mut kind = REG_VALUE_TYPE(0);
    let mut buf = [0u8; 1024];
    let mut size = buf.len() as u32;
    let err = unsafe { RegQueryValueExW(key, name, None, Some(&mut kind), Some(buf.as_mut_ptr()), Some(&mut size)) };
    if err.is_err() {
        return Vec::new();
    }
    // REG_SZ holds one string; REG_MULTI_SZ holds several separated by NULs.
    bytes_to_utf16(&buf[..size as usize])
        .split(|&u| u == 0)
        .filter(|s| !s.is_empty())
        .map(String::from_utf16_lossy)
        .collect()
}

fn interface_guids(set: &DeviceSet, data: &SP_DEVINFO_DATA) -> Vec<GUID> {
    let Ok(key) = (unsafe { SetupDiOpenDevRegKey(set.0, data, DICS_FLAG_GLOBAL.0, 0, DIREG_DEV, KEY_READ.0) }) else {
        return Vec::new();
    };
    let mut names = query_registry_strings(key, w!("DeviceInterfaceGUIDs"));
    names.extend(query_registry_strings(key, w!("DeviceInterfaceGUID")));
    let _ = unsafe { RegCloseKey(key) };
    names.iter().filter_map(|n| parse_guid(n)).collect()
}

/// Lists every connected Kinect USB device and composite function, whatever driver it uses.
pub fn scan() -> Result<Vec<UsbNode>> {
    let set = DeviceSet(unsafe { SetupDiGetClassDevsW(None, w!("USB"), None, DIGCF_ALLCLASSES | DIGCF_PRESENT)? });
    let mut nodes = Vec::new();
    for index in 0.. {
        let mut data = SP_DEVINFO_DATA { cbSize: size_of::<SP_DEVINFO_DATA>() as u32, ..Default::default() };
        if unsafe { SetupDiEnumDeviceInfo(set.0, index, &mut data) }.is_err() {
            break;
        }
        let instance_id = instance_id(&set, &data)?;
        let Some(identity) = identify(&instance_id) else { continue };
        nodes.push(UsbNode {
            product_id: identity.product_id,
            component: identity.component,
            interface: identity.interface,
            description: string_property(&set, &data, SPDRP_FRIENDLYNAME)
                .or_else(|| string_property(&set, &data, SPDRP_DEVICEDESC))
                .unwrap_or_default(),
            driver: string_property(&set, &data, SPDRP_SERVICE),
            problem: problem_code(&data),
            interface_guids: interface_guids(&set, &data),
            instance_id,
        });
    }
    nodes.sort_by_key(|n| (n.product_id, n.interface));
    Ok(nodes)
}

/// The path to open with `CreateFile` to reach `node` through WinUSB.
pub(crate) fn device_path(node: &UsbNode) -> Result<String> {
    for guid in &node.interface_guids {
        let set = DeviceSet(unsafe {
            SetupDiGetClassDevsW(Some(guid), PCWSTR::null(), None, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT)?
        });
        for index in 0.. {
            let mut iface =
                SP_DEVICE_INTERFACE_DATA { cbSize: size_of::<SP_DEVICE_INTERFACE_DATA>() as u32, ..Default::default() };
            if unsafe { SetupDiEnumDeviceInterfaces(set.0, None, guid, index, &mut iface) }.is_err() {
                break;
            }
            // Variable-length struct: a u32 cbSize followed by the NUL-terminated path.
            let mut storage = [0u64; 256];
            let detail = storage.as_mut_ptr().cast::<SP_DEVICE_INTERFACE_DETAIL_DATA_W>();
            let mut owner = SP_DEVINFO_DATA { cbSize: size_of::<SP_DEVINFO_DATA>() as u32, ..Default::default() };
            unsafe {
                (*detail).cbSize = size_of::<SP_DEVICE_INTERFACE_DETAIL_DATA_W>() as u32;
                SetupDiGetDeviceInterfaceDetailW(
                    set.0,
                    &iface,
                    Some(detail),
                    size_of_val(&storage) as u32,
                    None,
                    Some(&mut owner),
                )?;
            }
            if instance_id(&set, &owner)?.eq_ignore_ascii_case(&node.instance_id) {
                let path_units = unsafe {
                    let start = std::ptr::addr_of!((*detail).DevicePath).cast::<u16>();
                    let max = (size_of_val(&storage) - 4) / 2;
                    std::slice::from_raw_parts(start, max)
                };
                return Ok(utf16_until_nul(path_units));
            }
        }
    }
    Err(Error::WrongDriver {
        component: node.component.name(),
        driver: node.driver.clone().unwrap_or_else(|| "none".into()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_registry_guids() {
        let guid = parse_guid("{6B5F2D3A-8E41-4C7B-9A52-3F0D1E7C4B19}").unwrap();
        assert_eq!(guid, GUID::from_u128(0x6b5f2d3a_8e41_4c7b_9a52_3f0d1e7c4b19));
        assert_eq!(parse_guid("6b5f2d3a-8e41-4c7b-9a52-3f0d1e7c4b19"), Some(guid));
        assert_eq!(parse_guid("{not-a-guid}"), None);
    }
}
