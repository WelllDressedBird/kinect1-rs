//! Prints the distance to whatever is in front of the Kinect, ten times a second.
//!
//! cargo run -p kinect1 --example distance

use std::time::{Duration, Instant};

use kinect1::Kinect;
use kinect1::depth::{HEIGHT, NO_DEPTH_MM, WIDTH};

fn main() -> kinect1::Result<()> {
    let kinect = Kinect::open()?;
    let mut depth = kinect.start_depth()?;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(10) {
        let mm = depth.next_frame()?.to_mm(kinect.mm_table());
        // The nearest reading in a 20×20 pixel square in the middle of the image.
        let nearest = (HEIGHT / 2 - 10..HEIGHT / 2 + 10)
            .flat_map(|y| (WIDTH / 2 - 10..WIDTH / 2 + 10).map(move |x| y * WIDTH + x))
            .map(|i| mm[i])
            .filter(|&d| d != NO_DEPTH_MM)
            .min();
        match nearest {
            Some(d) => println!("{:>5} mm", d),
            None => println!("  too near or too far"),
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    Ok(())
}
