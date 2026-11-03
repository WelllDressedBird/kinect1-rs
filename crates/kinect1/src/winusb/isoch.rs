//! Isochronous IN streaming with WinUSB's ASAP read API (Windows 8.1 and later).
//!
//! One registered buffer is split into equal transfers that are kept queued with the driver;
//! each completed transfer is handed to the caller packet by packet, then queued again.

use std::ffi::c_void;
use std::time::Duration;

use windows::Win32::Devices::Usb::*;
use windows::Win32::Foundation::{CloseHandle, ERROR_IO_PENDING, WAIT_TIMEOUT};
use windows::Win32::System::IO::OVERLAPPED;
use windows::Win32::System::Threading::{CreateEventW, ResetEvent, WaitForSingleObject};
use windows_core::PCWSTR;

use super::WinUsbDevice;
use crate::{Error, Result};

struct Transfer {
    /// Boxed so its address stays fixed while the driver uses it.
    overlapped: Box<OVERLAPPED>,
    packets: Vec<USBD_ISO_PACKET_DESCRIPTOR>,
    offset: usize,
    pending: bool,
}

pub(crate) struct IsochReader {
    usb: WINUSB_INTERFACE_HANDLE,
    endpoint: u8,
    buffer: Box<[u8]>,
    buffer_handle: *mut c_void,
    transfers: Vec<Transfer>,
    transfer_len: usize,
    next: usize,
}

// WinUSB handles may be used from any thread.
unsafe impl Send for IsochReader {}

impl IsochReader {
    /// Starts reading `endpoint` with `transfers` queued transfers of `packets_per_transfer`
    /// packets each. At high speed `packets_per_transfer` must be a multiple of 8.
    pub fn new(
        device: &WinUsbDevice,
        endpoint: u8,
        bytes_per_interval: usize,
        transfers: usize,
        packets_per_transfer: usize,
    ) -> Result<Self> {
        let transfer_len = bytes_per_interval * packets_per_transfer;
        let mut buffer = vec![0u8; transfer_len * transfers].into_boxed_slice();
        let mut buffer_handle = std::ptr::null_mut();
        unsafe { WinUsb_RegisterIsochBuffer(device.handle(), endpoint, &mut buffer, &mut buffer_handle)? };
        // From here on, Drop releases everything.
        let mut reader = Self {
            usb: device.handle(),
            endpoint,
            buffer,
            buffer_handle,
            transfers: Vec::with_capacity(transfers),
            transfer_len,
            next: 0,
        };
        for i in 0..transfers {
            let event = unsafe { CreateEventW(None, true, false, PCWSTR::null())? };
            reader.transfers.push(Transfer {
                overlapped: Box::new(OVERLAPPED { hEvent: event, ..Default::default() }),
                packets: vec![USBD_ISO_PACKET_DESCRIPTOR::default(); packets_per_transfer],
                offset: i * transfer_len,
                pending: false,
            });
        }
        for i in 0..transfers {
            reader.submit(i, i > 0)?;
        }
        Ok(reader)
    }

    /// Queues transfer `index`. `continue_stream` schedules it right after the previous one.
    fn submit(&mut self, index: usize, continue_stream: bool) -> Result<()> {
        let t = &mut self.transfers[index];
        unsafe {
            ResetEvent(t.overlapped.hEvent)?;
            let result = WinUsb_ReadIsochPipeAsap(
                self.buffer_handle,
                t.offset as u32,
                self.transfer_len as u32,
                continue_stream,
                &mut t.packets,
                Some(&raw const *t.overlapped),
            );
            match result {
                Err(e) if e.code() != ERROR_IO_PENDING.to_hresult() => return Err(e.into()),
                _ => t.pending = true,
            }
        }
        Ok(())
    }

    /// Waits for the oldest queued transfer, passes each packet that arrived intact to
    /// `on_packet`, and queues the transfer again. Failed transfers just yield no packets.
    pub fn read(&mut self, timeout: Duration, mut on_packet: impl FnMut(&[u8])) -> Result<()> {
        let index = self.next;
        let ms = u32::try_from(timeout.as_millis()).unwrap_or(u32::MAX);
        let t = &mut self.transfers[index];
        if unsafe { WaitForSingleObject(t.overlapped.hEvent, ms) } == WAIT_TIMEOUT {
            return Err(Error::Timeout("isochronous data"));
        }
        t.pending = false;
        let mut len = 0u32;
        // A transfer fails as a whole when the device sent nothing in time, e.g. before its stream
        // is started or after the caller fell behind; the next one then starts a fresh stream.
        let ok = unsafe { WinUsb_GetOverlappedResult(self.usb, &raw const *t.overlapped, &mut len, false) }.is_ok();
        self.next = (index + 1) % self.transfers.len();

        let t = &self.transfers[index];
        for p in t.packets.iter().filter(|p| p.Status == 0 && p.Length > 0) {
            let start = t.offset + p.Offset as usize;
            on_packet(&self.buffer[start..start + p.Length as usize]);
        }
        self.submit(index, ok)
    }
}

impl Drop for IsochReader {
    fn drop(&mut self) {
        unsafe {
            let _ = WinUsb_AbortPipe(self.usb, self.endpoint);
            for t in &self.transfers {
                if t.pending {
                    let mut len = 0u32;
                    let _ = WinUsb_GetOverlappedResult(self.usb, &raw const *t.overlapped, &mut len, true);
                }
                let _ = CloseHandle(t.overlapped.hEvent);
            }
            let _ = WinUsb_UnregisterIsochBuffer(self.buffer_handle);
        }
    }
}
