# kinect1

A Rust SDK for the Xbox 360 Kinect (Kinect v1), aiming for depth, color, IR, tilt and LED
through Windows' built-in WinUSB driver, with an optional Kinect SDK 1.8 backend for skeletons.

**Status: early development.** Depth (640×480, 11-bit), color (640×480 RGB) and IR (640×488,
10-bit) stream at 30 fps; depth converts to millimetres and aligns with the color image using the
camera's own calibration; camera registers, LED and accelerometer work. Tilt does not move the motor yet. Not yet published. Needs Windows 8.1 or later.

## Setup (WinUSB backend)

The camera must use the WinUSB driver instead of Microsoft's Kinect camera driver:

1. Download [Zadig](https://zadig.akeo.ie/) and run it.
2. Options → List All Devices.
3. Pick the device with USB ID `045E 02AE` (named "Kinect for Windows Camera" or "Xbox NUI Camera").
   Check the USB ID boxes rather than the name: the audio device (`045E 02AD`, "Xbox Kinect Audio…")
   is easy to pick by mistake, and replacing its driver stops the motor and LED from working.
4. Choose **WinUSB** and click **Replace Driver**.

Only the camera changes; the microphone array keeps working as a Windows audio device.
To go back to Microsoft's driver, open Device Manager, find the camera, choose
Uninstall device and tick "Attempt to remove the driver for this device", then scan for hardware changes.

## Try it

```powershell
cargo run -p kinect1-tool -- info
```

Lists every Kinect USB device with its driver and, for devices on WinUSB, their USB descriptors.

```powershell
cargo run -p kinect1-tool -- motor             # tilt angle, motor status, accelerometer
cargo run -p kinect1-tool -- motor led green   # off, green, red, yellow, blink-green, blink-red-yellow
cargo run -p kinect1-tool -- motor tilt 10     # -27..=27 degrees
```

```powershell
cargo run -p kinect1-tool -- depth depth.pgm   # stream depth for 3 s, save the last frame as an image
cargo run -p kinect1-tool -- color color.ppm   # the same for color (640×480 RGB)
cargo run -p kinect1-tool -- ir ir.pgm         # the same for IR (640×488, 10-bit)
cargo run -p kinect1-tool -- both              # depth and color at once, on two threads
cargo run -p kinect1-tool -- calibration       # the camera's depth calibration
cargo run -p kinect1-tool -- align align.ppm   # depth aligned onto a color frame, as an overlay
```

Color and IR share an endpoint, so only one of them can run at a time; depth runs alongside either.

```powershell
cargo run -p kinect1-tool -- camera reg 0x15         # read a camera register (0x15 = IR brightness)
cargo run -p kinect1-tool -- camera reg 0x15 0x1e    # write one; needs the camera on WinUSB
```

On Kinect for Windows and Xbox model 1473 units the motor is driven through the audio device, which
the Kinect for Windows runtime already binds to WinUSB, so these work without Zadig. Original model
1414 units (separate motor device, `045E 02B0`) are not supported yet.

## Using it in your project

Converting frames (unpacking depth, demosaicing color) is too slow at 30 fps without optimizations,
so frames get dropped in debug builds. Optimize just this crate in your `Cargo.toml`:

```toml
[profile.dev.package.kinect1]
opt-level = 3
```

## License

MIT OR Apache-2.0. Protocol knowledge comes from the OpenKinect project's documentation.
