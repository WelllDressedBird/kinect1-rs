//! Command-line tool for inspecting and testing a Kinect v1. Run without arguments for usage.

use std::process::ExitCode;

use std::time::{Duration, Instant};

use kinect1::descriptors::TransferType;
use kinect1::motor::Led;
use kinect1::winusb::{Camera, Motor, UsbNode, WinUsbDevice, scan};
use kinect1::{Component, Error, Result};

const USAGE: &str = "Usage:
  kinect1-tool info                 list Kinect USB devices and their descriptors
  kinect1-tool motor                show tilt angle and accelerometer
  kinect1-tool motor tilt <degrees> tilt to -27..=27 degrees from level
  kinect1-tool motor led <state>    set the LED: off, green, red, yellow, blink-green, blink-red-yellow
  kinect1-tool camera reg <reg>     read a camera register (hex, e.g. 0x15)
  kinect1-tool camera reg <reg> <value>  write a camera register";

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    let result = match args[..] {
        [] | ["info"] => info(),
        ["motor"] => motor_state(),
        ["motor", "tilt", degrees] => match degrees.parse() {
            Ok(degrees) => tilt(degrees),
            Err(_) => Err(Error::InvalidArgument(format!("{degrees:?} is not a whole number of degrees"))),
        },
        ["motor", "led", state] => led(state),
        ["camera", "reg", register] => parse_hex(register).and_then(read_register),
        ["camera", "reg", register, value] => {
            parse_hex(register).and_then(|r| parse_hex(value).and_then(|v| write_register(r, v)))
        }
        _ => {
            eprintln!("{USAGE}");
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

fn print_state(motor: &mut Motor) -> Result<()> {
    let state = motor.state()?;
    let a = state.accelerometer;
    let [x, y, z] = a.meters_per_second_squared();
    println!(
        "tilt {:>3}° {:<9} accelerometer x {:>5} y {:>5} z {:>5} counts = ({x:+.2}, {y:+.2}, {z:+.2}) m/s²",
        state.angle,
        format!("{:?}", state.status),
        a.x,
        a.y,
        a.z
    );
    Ok(())
}

fn motor_state() -> Result<()> {
    print_state(&mut Motor::open()?)
}

fn tilt(degrees: i8) -> Result<()> {
    let mut motor = Motor::open()?;
    motor.set_tilt(degrees)?;
    // The command returns at once; report the angle while the motor moves.
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(3) {
        print_state(&mut motor)?;
        std::thread::sleep(Duration::from_millis(250));
    }
    Ok(())
}

fn led(state: &str) -> Result<()> {
    let led = match state {
        "off" => Led::Off,
        "green" => Led::Green,
        "red" => Led::Red,
        "yellow" => Led::Yellow,
        "blink-green" => Led::BlinkGreen,
        "blink-red-yellow" => Led::BlinkRedYellow,
        _ => return Err(Error::InvalidArgument(format!("unknown LED state {state:?}"))),
    };
    Motor::open()?.set_led(led)
}

fn parse_hex(text: &str) -> Result<u16> {
    let digits = text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")).unwrap_or(text);
    u16::from_str_radix(digits, 16).map_err(|_| Error::InvalidArgument(format!("{text:?} is not a 16-bit hex number")))
}

fn read_register(register: u16) -> Result<()> {
    let value = Camera::open()?.read_register(register)?;
    println!("register {register:#06x} = {value:#06x} ({value})");
    Ok(())
}

fn write_register(register: u16, value: u16) -> Result<()> {
    let mut camera = Camera::open()?;
    camera.write_register(register, value)?;
    let read_back = camera.read_register(register)?;
    println!("register {register:#06x} <= {value:#06x}, reads back {read_back:#06x}");
    Ok(())
}
