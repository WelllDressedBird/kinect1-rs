//! `kinect1-tool info`: lists Kinect USB devices, their drivers and, for WinUSB devices, their descriptors.

use std::process::ExitCode;

use kinect1::descriptors::TransferType;
use kinect1::winusb::{UsbNode, WinUsbDevice, scan};
use kinect1::{Component, Result};

fn main() -> ExitCode {
    let command = std::env::args().nth(1);
    let result = match command.as_deref() {
        None | Some("info") => info(),
        Some(other) => {
            eprintln!("unknown command {other:?}\nUsage: kinect1-tool info");
            return ExitCode::FAILURE;
        }
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

fn info() -> Result<()> {
    let nodes = scan()?;
    if nodes.is_empty() {
        println!("No Kinect found. Check that it is plugged into USB and its power adapter.");
        return Ok(());
    }

    println!("Kinect USB devices:");
    for n in &nodes {
        let function = n.interface.map(|i| format!(" interface {i}")).unwrap_or_default();
        let problem = n.problem.map(|p| format!(", problem code {p}")).unwrap_or_default();
        println!(
            "  {:<6} {:04X}{function}  {:?}  driver: {}{problem}",
            n.component.name(),
            n.product_id,
            n.description,
            n.driver.as_deref().unwrap_or("none"),
        );
    }

    for node in nodes.iter().filter(|n| n.is_winusb()) {
        println!();
        if let Err(e) = describe(node) {
            println!("  could not read descriptors: {e}");
        }
    }

    if let Some(camera) = nodes.iter().find(|n| n.component == Component::Camera && !n.is_winusb()) {
        println!(
            "\nThe camera uses the {:?} driver, so the WinUSB backend can't open it yet.\n\
             To switch it: run Zadig, Options > List All Devices, pick the device with USB ID 045E 02AE\n\
             (\"Kinect for Windows Camera\" or \"Xbox NUI Camera\"),\n\
             choose WinUSB and click Replace Driver. Only that device changes; the microphone keeps its driver.",
            camera.driver.as_deref().unwrap_or("none")
        );
    }
    Ok(())
}

fn describe(node: &UsbNode) -> Result<()> {
    let device = WinUsbDevice::open(node)?;
    let d = device.device_descriptor()?;
    println!("{} {:04X}:{:04X} (USB {:x}.{:02x}, release {:x}.{:02x})", node.component.name(), d.vendor_id, d.product_id,
        d.usb_version >> 8, d.usb_version & 0xff, d.device_version >> 8, d.device_version & 0xff);
    for (label, index) in [("manufacturer", d.manufacturer_index), ("product", d.product_index), ("serial", d.serial_index)] {
        if let Ok(Some(s)) = device.string(index) {
            println!("  {label}: {s}");
        }
    }
    let config = device.configuration()?;
    println!("  configuration {}, {} mA, current alternate setting {}", config.value, config.max_power_ma, device.alternate_setting()?);
    for s in &config.settings {
        println!("  interface {} alt {}: class {:02X}/{:02X}/{:02X}", s.number, s.alternate, s.class, s.subclass, s.protocol);
        for e in &s.endpoints {
            let dir = if e.is_in() { "IN " } else { "OUT" };
            let kind = match e.transfer_type {
                TransferType::Isochronous => "isochronous",
                TransferType::Bulk => "bulk",
                TransferType::Interrupt => "interrupt",
                TransferType::Control => "control",
            };
            println!(
                "    endpoint 0x{:02X} {dir} {kind:<11} {} bytes x {} per microframe = {} bytes, interval {}",
                e.address, e.max_packet_size, e.transactions_per_microframe, e.bytes_per_interval(), e.interval
            );
        }
    }
    Ok(())
}
