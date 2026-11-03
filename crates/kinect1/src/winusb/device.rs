//! An open WinUSB handle to one Kinect device.

use std::time::Duration;

use windows::Win32::Devices::Usb::*;
use windows::Win32::Foundation::{CloseHandle, GENERIC_READ, GENERIC_WRITE, HANDLE};
use windows::Win32::Storage::FileSystem::{
    CreateFileW, FILE_ATTRIBUTE_NORMAL, FILE_FLAG_OVERLAPPED, FILE_SHARE_READ, FILE_SHARE_WRITE, OPEN_EXISTING,
};
use windows_core::HSTRING;

use super::setup::{UsbNode, device_path};
use crate::descriptors::{Configuration, DeviceDescriptor};
use crate::{Error, Result};

const LANGUAGE_EN_US: u16 = 0x0409;

pub struct WinUsbDevice {
    file: HANDLE,
    usb: WINUSB_INTERFACE_HANDLE,
}

// WinUSB handles may be used from any thread.
unsafe impl Send for WinUsbDevice {}
unsafe impl Sync for WinUsbDevice {}

impl WinUsbDevice {
    pub fn open(node: &UsbNode) -> Result<Self> {
        if !node.is_winusb() {
            return Err(Error::WrongDriver {
                component: node.component.name(),
                driver: node.driver.clone().unwrap_or_else(|| "none".into()),
            });
        }
        let path = device_path(node)?;
        unsafe {
            let file = CreateFileW(
                &HSTRING::from(path),
                (GENERIC_READ | GENERIC_WRITE).0,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                None,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                None,
            )?;
            let mut usb = WINUSB_INTERFACE_HANDLE::default();
            if let Err(e) = WinUsb_Initialize(file, &mut usb) {
                let _ = CloseHandle(file);
                return Err(e.into());
            }
            Ok(Self { file, usb })
        }
    }

    fn descriptor(&self, kind: u32, index: u8, language: u16, max_len: usize) -> Result<Vec<u8>> {
        let mut buf = vec![0u8; max_len];
        let mut len = 0u32;
        unsafe { WinUsb_GetDescriptor(self.usb, kind as u8, index, language, Some(&mut buf), &mut len)? };
        buf.truncate(len as usize);
        Ok(buf)
    }

    pub fn device_descriptor(&self) -> Result<DeviceDescriptor> {
        let bytes = self.descriptor(USB_DEVICE_DESCRIPTOR_TYPE, 0, 0, DeviceDescriptor::LEN)?;
        DeviceDescriptor::parse(&bytes).ok_or_else(|| Error::Protocol("malformed device descriptor".into()))
    }

    pub fn configuration(&self) -> Result<Configuration> {
        let malformed = || Error::Protocol("malformed configuration descriptor".into());
        let header = self.descriptor(USB_CONFIGURATION_DESCRIPTOR_TYPE, 0, 0, Configuration::HEADER_LEN)?;
        let total = Configuration::total_length(&header).ok_or_else(malformed)?;
        let bytes = self.descriptor(USB_CONFIGURATION_DESCRIPTOR_TYPE, 0, 0, total.into())?;
        Configuration::parse(&bytes).ok_or_else(malformed)
    }

    /// A string descriptor such as the serial number. Index 0 means "no string".
    pub fn string(&self, index: u8) -> Result<Option<String>> {
        if index == 0 {
            return Ok(None);
        }
        let bytes = self.descriptor(USB_STRING_DESCRIPTOR_TYPE, index, LANGUAGE_EN_US, 255)?;
        let units: Vec<u16> = bytes.get(2..).unwrap_or_default().chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]])).collect();
        Ok(Some(String::from_utf16_lossy(&units)))
    }

    pub fn alternate_setting(&self) -> Result<u8> {
        let mut setting = 0u8;
        unsafe { WinUsb_GetCurrentAlternateSetting(self.usb, &mut setting)? };
        Ok(setting)
    }

    /// Makes transfers on `endpoint` fail with a timeout error after `timeout`.
    pub fn set_timeout(&self, endpoint: u8, timeout: Duration) -> Result<()> {
        let ms = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);
        unsafe {
            WinUsb_SetPipePolicy(self.usb, endpoint, PIPE_TRANSFER_TIMEOUT, size_of::<u32>() as u32, (&raw const ms).cast())?
        };
        Ok(())
    }

    /// Discards data the device has sent on IN `endpoint` that nobody has read yet.
    pub fn flush(&self, endpoint: u8) -> Result<()> {
        unsafe { WinUsb_FlushPipe(self.usb, endpoint)? };
        Ok(())
    }

    fn control(&self, request_type: u8, request: u8, value: u16, index: u16, buf: &mut [u8]) -> Result<usize> {
        let setup = WINUSB_SETUP_PACKET {
            RequestType: request_type,
            Request: request,
            Value: value,
            Index: index,
            Length: u16::try_from(buf.len()).map_err(|_| Error::InvalidArgument("control transfer over 64 KiB".into()))?,
        };
        let mut len = 0u32;
        let buf = (!buf.is_empty()).then_some(buf);
        unsafe { WinUsb_ControlTransfer(self.usb, setup, buf, Some(&mut len), None)? };
        Ok(len as usize)
    }

    /// Control transfer from host to device, e.g. request type 0x40 for a vendor request.
    pub fn control_out(&self, request_type: u8, request: u8, value: u16, index: u16, data: &[u8]) -> Result<usize> {
        self.control(request_type & 0x7f, request, value, index, &mut data.to_vec())
    }

    /// Control transfer from device to host, e.g. request type 0xC0 for a vendor request.
    /// Returns the number of bytes received.
    pub fn control_in(&self, request_type: u8, request: u8, value: u16, index: u16, buf: &mut [u8]) -> Result<usize> {
        self.control(request_type | 0x80, request, value, index, buf)
    }

    /// Bulk or interrupt OUT transfer. Returns the number of bytes sent.
    pub fn write(&self, endpoint: u8, data: &[u8]) -> Result<usize> {
        let mut len = 0u32;
        unsafe { WinUsb_WritePipe(self.usb, endpoint, data, Some(&mut len), None)? };
        Ok(len as usize)
    }

    /// Bulk or interrupt IN transfer. Returns the number of bytes received.
    pub fn read(&self, endpoint: u8, buf: &mut [u8]) -> Result<usize> {
        let mut len = 0u32;
        unsafe { WinUsb_ReadPipe(self.usb, endpoint, Some(buf), Some(&mut len), None)? };
        Ok(len as usize)
    }
}

impl Drop for WinUsbDevice {
    fn drop(&mut self) {
        unsafe {
            let _ = WinUsb_Free(self.usb);
            let _ = CloseHandle(self.file);
        }
    }
}
