//! Captures one depth and color frame and writes them as a colored point cloud in PLY format,
//! which MeshLab, CloudCompare or Blender can open.
//!
//! cargo run -p kinect1 --example point_cloud -- cloud.ply

use std::fmt::Write as _;

use kinect1::Kinect;
use kinect1::depth::{NO_DEPTH_MM, WIDTH};

fn main() -> kinect1::Result<()> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "cloud.ply".into());
    let kinect = Kinect::open()?;
    let mut depth_stream = kinect.start_depth()?;
    let mut color_stream = kinect.start_color()?;
    // Give the color camera's auto exposure a second to settle.
    for _ in 0..30 {
        color_stream.next_frame()?;
    }
    let depth_mm = depth_stream.next_frame()?.to_mm(kinect.mm_table());
    let color = color_stream.next_frame()?;
    let colors = kinect.registration().color_for_depth(&depth_mm, &color.rgb);

    let mut points = String::new();
    let mut count = 0;
    for (i, &mm) in depth_mm.iter().enumerate() {
        let rgb = &colors[3 * i..3 * i + 3];
        // Skip pixels without depth, and those the color camera can't see.
        if mm == NO_DEPTH_MM || rgb == [0, 0, 0] {
            continue;
        }
        let [x, y, z] = kinect.calibration().camera_to_world(i % WIDTH, i / WIDTH, mm);
        // PLY viewers expect y up, so flip the image's downward y.
        writeln!(points, "{x:.1} {:.1} {z:.1} {} {} {}", -y, rgb[0], rgb[1], rgb[2]).unwrap();
        count += 1;
    }

    let header = format!(
        "ply\nformat ascii 1.0\nelement vertex {count}\n\
         property float x\nproperty float y\nproperty float z\n\
         property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    );
    std::fs::write(&path, header + &points).map_err(|e| kinect1::Error::InvalidArgument(format!("{path}: {e}")))?;
    println!("wrote {count} points to {path}");
    Ok(())
}
