# kinect1

A Rust SDK for the Xbox 360 Kinect (Kinect v1), aiming for depth, color, IR, tilt and LED
through Windows' built-in WinUSB driver, with an optional Kinect SDK 1.8 backend for skeletons.

**Status: early development (milestone M0, discovery).** Not yet published.

## Setup (WinUSB backend)

The camera must use the WinUSB driver instead of Microsoft's Kinect camera driver:

1. Download [Zadig](https://zadig.akeo.ie/) and run it.
2. Options → List All Devices.
3. Pick the device with USB ID `045E 02AE` (named "Kinect for Windows Camera" or "Xbox NUI Camera").
4. Choose **WinUSB** and click **Replace Driver**.

Only the camera changes; the microphone array keeps working as a Windows audio device.
To go back to Microsoft's driver, open Device Manager, find the camera, choose
Uninstall device and tick "Attempt to remove the driver for this device", then scan for hardware changes.

## Try it

```powershell
cargo run -p kinect1-tool -- info
```

Lists every Kinect USB device with its driver and, for devices on WinUSB, their USB descriptors.

## License

MIT OR Apache-2.0. Protocol knowledge comes from the OpenKinect project's documentation.
