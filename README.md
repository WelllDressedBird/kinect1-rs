# kinect1

A Rust library for the Xbox 360 Kinect and Kinect for Windows v1 on Windows, talking to the sensor
directly through Windows' built-in WinUSB driver: no Kinect SDK needed for the cameras.

- Depth at 640×480, 30 fps, as raw values or millimetres using the camera's own calibration
- Color at 640×480 (30 fps) or 1280×1024 (up to 15 fps), demosaiced to RGB
- IR at 640×488 (30 fps) or 1280×1024
- Depth aligned to the color image, and 3D points in millimetres
- Tilt motor, LED and accelerometer
- Each stream reads USB on its own thread; depth runs alongside color or IR

```rust
use kinect1::Kinect;

let kinect = Kinect::open()?;              // camera, its calibration, and the motor if reachable
let mut depth = kinect.start_depth()?;
let mut color = kinect.start_color()?;

let depth_mm = depth.next_frame()?.to_mm(kinect.mm_table());
let rgb = color.next_frame()?.rgb;
let depth_on_color = kinect.registration().depth_to_color(&depth_mm);
kinect.motor().unwrap().tilt(10)?;
```

Status: early, version 0.1. The API may still change. Needs Windows 8.1 or later.

## Hardware

| Model | Camera streams | Tilt, LED, accelerometer |
|---|---|---|
| Xbox 360 Kinect, model 1473 | tested | tested |
| Kinect for Windows (model 1517) | expected to work, untested | expected to work, untested; near mode untested |
| Xbox 360 Kinect, model 1414 | expected to work, untested | implemented from libfreenect, untested |

The model number is on the bottom of the base. If you try an untested model, please open an issue
with what worked and the output of `cargo run -p kinect1-tool -- info`.

## Setup

**Camera.** It must use the WinUSB driver instead of Microsoft's camera driver:

1. Download [Zadig](https://zadig.akeo.ie/) and run it.
2. Options → List All Devices.
3. Pick the device with USB ID `045E 02AE` (`045E 02BF` on a Kinect for Windows), named
   "Kinect for Windows Camera" or "Xbox NUI Camera". Check the USB ID boxes rather than the name:
   the audio device ("Xbox Kinect Audio…") is easy to pick by mistake, and replacing its driver
   stops the motor and LED from working.
4. Choose **WinUSB** and click **Replace Driver**.

Only the camera changes; the microphones keep working as a Windows audio device. To go back to
Microsoft's driver, open Device Manager, find the camera, choose Uninstall device and tick
"Attempt to remove the driver for this device", then scan for hardware changes.

**Motor, LED and accelerometer.** On model 1473 and Kinect for Windows units these go through the
audio device, which needs firmware that Microsoft's
[Kinect for Windows Runtime 1.8](https://www.microsoft.com/en-us/download/details.aspx?id=40277)
loads into it. Install the runtime, leave the audio device on Microsoft's driver and the
KinectManagement service running. On a model 1414 the motor is its own device (`045E 02B0`); give
it WinUSB with Zadig the same way as the camera.

## Using it

```powershell
cargo add kinect1
```

Converting frames (unpacking depth, demosaicing color) runs in `next_frame` on your thread, and
unoptimized it is slow: color only reaches about 12 fps. Frames are skipped rather than corrupted
when you fall behind, but to get full frame rates in debug builds, optimize just this crate:

```toml
[profile.dev.package.kinect1]
opt-level = 3
```

Color and IR share a USB endpoint, so only one of them can run at a time; depth runs alongside
either, except 1280×1024 IR.

Examples:

```powershell
cargo run -p kinect1 --example distance                  # distance to what's in front, live
cargo run -p kinect1 --example point_cloud -- cloud.ply  # a colored 3D point cloud (MeshLab, Blender)
```

## Command-line tool

`kinect1-tool` (in this repository, not published) inspects and tests a Kinect:

```powershell
cargo run -p kinect1-tool -- info                 # Kinect USB devices, their drivers and descriptors
cargo run -p kinect1-tool -- depth depth.pgm      # stream depth for 3 s, save the last frame
cargo run -p kinect1-tool -- color --high c.ppm   # the same for color; --high for 1280×1024
cargo run -p kinect1-tool -- ir ir.pgm            # the same for IR
cargo run -p kinect1-tool -- both                 # depth and color at once, on two threads
cargo run -p kinect1-tool -- align align.ppm      # depth aligned onto a color frame, as an overlay
cargo run -p kinect1-tool -- calibration          # the camera's depth calibration
cargo run -p kinect1-tool -- motor tilt 10        # tilt, waiting until the motor stops
cargo run -p kinect1-tool -- motor led green      # off, green, red, yellow, blink-green, blink-red-yellow
cargo run -p kinect1-tool -- camera reg 0x15      # read (or, with a value, write) a camera register
```

## Troubleshooting

- **The camera shows problem code 39 in Device Manager / `info`.** Microsoft's camera driver is
  blocked, usually by Windows' Memory Integrity setting. That's fine: put the camera on WinUSB as
  above.
- **Tilt fails with "the tilt motor did not move".** The motor only runs while the camera has a
  working driver. Put the camera on WinUSB.
- **"the Kinect audio firmware is not running".** The audio device shows as `045E 02AD`
  (`02BE` on Kinect for Windows): start the KinectManagement service and replug the Kinect. If
  the audio device has Zadig's driver, remove it (Device Manager → Uninstall device, tick
  "Attempt to remove the driver") so Microsoft's comes back.
- **A few frames are dropped right after a stream starts.** Normal: the camera sends some
  incomplete packets while it starts up.

## Contributing

Issues and pull requests are welcome, especially reports from untested hardware. The protocol
code (`camera`, `depth`, `motor`, `registration`, `stream`, `video`) has no I/O and is unit
tested; device access lives in `winusb`. Please run `cargo fmt`, `cargo clippy --all-targets` and
`cargo test` before sending changes.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT)
at your option. Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this crate by you, as defined in the Apache-2.0 license, shall be dual licensed as
above, without any additional terms or conditions.

## Acknowledgements

The USB protocol, register values and the depth-to-millimetre and depth-to-color algorithms come
from [libfreenect](https://github.com/OpenKinect/libfreenect) by the OpenKinect project, used
under the Apache License 2.0; `registration.rs` is a port of its `registration.c`. See
[NOTICE](NOTICE).
