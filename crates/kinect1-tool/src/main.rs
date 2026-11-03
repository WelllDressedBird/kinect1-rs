//! Command-line tool for inspecting and testing a Kinect v1. Run without arguments for usage.

use std::process::ExitCode;

use std::time::{Duration, Instant};

use kinect1::depth::{self, DepthFrame, MmTable};
use kinect1::video::{self, ColorFrame, IrFrame, Resolution};
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
  kinect1-tool camera reg <reg> <value>  write a camera register
  kinect1-tool depth [out.pgm]      stream depth for 3 s, optionally saving the last frame
  kinect1-tool color [--high] [out.ppm]  the same for color; --high for 1280×1024 at 15 fps
  kinect1-tool ir [--high] [out.pgm]     the same for IR
  kinect1-tool both                 stream depth and color at the same time
  kinect1-tool calibration          show the camera's depth calibration
  kinect1-tool align <out.ppm>      overlay depth, aligned to it, on a color frame";

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
        ["depth"] => depth(None),
        ["depth", path] => depth(Some(path)),
        ["color"] => color(Resolution::Medium, None),
        ["color", "--high"] => color(Resolution::High, None),
        ["color", "--high", path] => color(Resolution::High, Some(path)),
        ["color", path] => color(Resolution::Medium, Some(path)),
        ["ir"] => ir(Resolution::Medium, None),
        ["ir", "--high"] => ir(Resolution::High, None),
        ["ir", "--high", path] => ir(Resolution::High, Some(path)),
        ["ir", path] => ir(Resolution::Medium, Some(path)),
        ["both"] => both(),
        ["calibration"] => calibration(),
        ["align", path] => align(path),
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

fn print_state(motor: &Motor) -> Result<()> {
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
    print_state(&Motor::open()?)
}

fn tilt(degrees: i8) -> Result<()> {
    let motor = Motor::open()?;
    print_state(&motor)?;
    motor.tilt(degrees)?;
    print_state(&motor)
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
    let camera = Camera::open()?;
    camera.write_register(register, value)?;
    let read_back = camera.read_register(register)?;
    println!("register {register:#06x} <= {value:#06x}, reads back {read_back:#06x}");
    Ok(())
}

/// Reads frames for three seconds, printing `describe` for every 15th, then reports the rate.
/// Returns the last frame.
fn capture<F>(name: &str, mut next_frame: impl FnMut() -> Result<F>, describe: impl Fn(&F) -> String) -> Result<Option<F>> {
    let start = Instant::now();
    let mut frames = 0;
    let mut last = None;
    while start.elapsed() < Duration::from_secs(3) {
        let frame = next_frame()?;
        frames += 1;
        if frames % 15 == 1 {
            println!("{name} frame {frames:>3}  {}", describe(&frame));
        }
        last = Some(frame);
    }
    let secs = start.elapsed().as_secs_f64();
    println!("{name}: {frames} frames in {secs:.1} s = {:.1} fps", frames as f64 / secs);
    Ok(last)
}

fn describe_depth(frame: &DepthFrame, table: &MmTable) -> String {
    let mm: Vec<u16> = frame.to_mm(table).into_iter().filter(|&d| d != depth::NO_DEPTH_MM).collect();
    let centre = frame.get(depth::WIDTH / 2, depth::HEIGHT / 2);
    format!(
        "timestamp {:>10}  centre {centre:>4} = {:>4} mm  valid {:>5.1}%  range {}..={} mm",
        frame.timestamp,
        table.mm(centre),
        100.0 * mm.len() as f64 / frame.data.len() as f64,
        mm.iter().min().unwrap_or(&0),
        mm.iter().max().unwrap_or(&0),
    )
}

fn describe_color(frame: &ColorFrame) -> String {
    let mean = |c: usize| frame.rgb.iter().skip(c).step_by(3).map(|&v| u64::from(v)).sum::<u64>() / (frame.rgb.len() / 3) as u64;
    format!(
        "timestamp {:>10}  centre {:?}  mean rgb [{}, {}, {}]",
        frame.timestamp,
        frame.get(frame.width / 2, frame.height / 2),
        mean(0),
        mean(1),
        mean(2),
    )
}

fn describe_ir(frame: &IrFrame) -> String {
    let mean = frame.data.iter().map(|&v| u64::from(v)).sum::<u64>() / frame.data.len() as u64;
    let max = frame.data.iter().max().unwrap_or(&0);
    format!("timestamp {:>10}  centre {:>4}  mean {mean}  max {max}", frame.timestamp, frame.get(frame.width / 2, frame.height / 2))
}

fn save(path: Option<&str>, write: impl FnOnce(&str) -> std::io::Result<()>) -> Result<()> {
    if let Some(path) = path {
        write(path).map_err(|e| Error::InvalidArgument(format!("could not write {path}: {e}")))?;
        println!("saved the last frame to {path}");
    }
    Ok(())
}

fn depth(path: Option<&str>) -> Result<()> {
    let camera = Camera::open()?;
    let table = camera.depth_calibration()?.mm_table();
    let mut stream = camera.start_depth()?;
    let last = capture("depth", || stream.next_frame(), |f| describe_depth(f, &table))?;
    println!("depth: {} frames dropped, {} skipped", stream.dropped_frames(), stream.skipped_frames());
    match last {
        Some(frame) => save(path, |p| save_depth(p, &frame)),
        None => Ok(()),
    }
}

fn color(resolution: Resolution, path: Option<&str>) -> Result<()> {
    let camera = Camera::open()?;
    let mut stream = camera.start_color_at(resolution)?;
    let last = capture("color", || stream.next_frame(), describe_color)?;
    println!("color: {} frames dropped, {} skipped", stream.dropped_frames(), stream.skipped_frames());
    match last {
        Some(frame) => save(path, |p| save_color(p, &frame)),
        None => Ok(()),
    }
}

fn ir(resolution: Resolution, path: Option<&str>) -> Result<()> {
    let camera = Camera::open()?;
    let mut stream = camera.start_ir_at(resolution)?;
    let last = capture("ir", || stream.next_frame(), describe_ir)?;
    println!("ir: {} frames dropped, {} skipped", stream.dropped_frames(), stream.skipped_frames());
    match last {
        Some(frame) => save(path, |p| save_ir(p, &frame)),
        None => Ok(()),
    }
}

fn calibration() -> Result<()> {
    let c = Camera::open()?.depth_calibration()?;
    println!("IR camera to projector: {} cm", c.emitter_distance);
    println!("IR camera to color camera: {} cm", c.color_camera_distance);
    println!("reference distance: {}", c.reference_distance);
    println!("reference pixel size: {} mm", c.reference_pixel_size);
    println!("constant shift: {}", c.const_shift);
    for raw in [400, 600, 800, 900, 1000] {
        println!("raw {raw:>4} = {:?} mm", c.raw_to_mm(raw));
    }
    Ok(())
}

/// Captures depth and color together and writes the color frame with the registered depth
/// over it: near is red, far is blue, no reading leaves the colour dimmed.
fn align(path: &str) -> Result<()> {
    let camera = Camera::open()?;
    let table = camera.depth_calibration()?.mm_table();
    let registration = camera.registration()?;
    let mut depth_stream = camera.start_depth()?;
    let mut color_stream = camera.start_color()?;
    // Let auto exposure settle, then take one frame of each.
    for _ in 0..30 {
        depth_stream.next_frame()?;
        color_stream.next_frame()?;
    }
    let depth = depth_stream.next_frame()?;
    let color = color_stream.next_frame()?;

    let registered = registration.depth_to_color(&depth.to_mm(&table));
    let covered = registered.iter().filter(|&&mm| mm != depth::NO_DEPTH_MM).count();
    println!("{:.1}% of colour pixels have a depth", 100.0 * covered as f64 / registered.len() as f64);
    let (near, far) = (500.0, 3000.0);
    let pixels = registered.iter().zip(color.rgb.chunks(3)).flat_map(|(&mm, rgb)| {
        if mm == depth::NO_DEPTH_MM {
            return [rgb[0] / 3, rgb[1] / 3, rgb[2] / 3];
        }
        let t = ((f64::from(mm) - near) / (far - near)).clamp(0.0, 1.0);
        let tint = [255.0 * (1.0 - t), 0.0, 255.0 * t];
        std::array::from_fn(|c| ((f64::from(rgb[c]) + tint[c]) / 2.0) as u8)
    });
    save(Some(path), |p| pnm(p, "P6", video::COLOR_WIDTH, video::COLOR_HEIGHT, pixels))
}

/// Streams depth and color at the same time, each on its own thread.
fn both() -> Result<()> {
    let camera = Camera::open()?;
    let table = camera.depth_calibration()?.mm_table();
    std::thread::scope(|scope| {
        let depth = scope.spawn(|| -> Result<()> {
            let mut stream = camera.start_depth()?;
            capture("depth", || stream.next_frame(), |f| describe_depth(f, &table))?;
            println!("depth: {} frames dropped, {} skipped", stream.dropped_frames(), stream.skipped_frames());
            Ok(())
        });
        let color = scope.spawn(|| -> Result<()> {
            let mut stream = camera.start_color()?;
            capture("color", || stream.next_frame(), describe_color)?;
            println!("color: {} frames dropped, {} skipped", stream.dropped_frames(), stream.skipped_frames());
            Ok(())
        });
        depth.join().expect("depth thread panicked")?;
        color.join().expect("color thread panicked")
    })
}

fn pnm(path: &str, magic: &str, width: usize, height: usize, pixels: impl IntoIterator<Item = u8>) -> std::io::Result<()> {
    let mut bytes = format!("{magic}
{width} {height}
255
").into_bytes();
    bytes.extend(pixels);
    std::fs::write(path, bytes)
}

/// Writes `frame` as an 8-bit greyscale PGM: near is bright, no reading is black.
fn save_depth(path: &str, frame: &DepthFrame) -> std::io::Result<()> {
    let pixels = frame.data.iter().map(|&d| if d == depth::NO_DEPTH { 0 } else { 255 - (d >> 3).min(255) as u8 });
    pnm(path, "P5", depth::WIDTH, depth::HEIGHT, pixels)
}

/// Writes `frame` as a PPM.
fn save_color(path: &str, frame: &ColorFrame) -> std::io::Result<()> {
    pnm(path, "P6", frame.width, frame.height, frame.rgb.iter().copied())
}

/// Writes `frame` as an 8-bit greyscale PGM.
fn save_ir(path: &str, frame: &IrFrame) -> std::io::Result<()> {
    pnm(path, "P5", frame.width, frame.height, frame.data.iter().map(|&v| (v >> 2) as u8))
}
