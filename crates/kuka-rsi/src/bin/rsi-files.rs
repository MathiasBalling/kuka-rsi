//! Write the robot-side files into a folder, for example on a USB stick for the controller.

use clap::Parser;
use kuka_rsi::controller::{FILES, write_files};

#[derive(Parser)]
#[command(
    name = "rsi-files",
    about = "Write the controller files (RSI contexts, Ethernet configs, KRL programs) into a folder",
    after_help = "DIR/SensorInterface/ goes to C:\\KRC\\ROBOTER\\Config\\User\\Common\\SensorInterface\\ and \
                  DIR/Program/ (CRLF line endings) to KRC:\\R1\\Program\\; the licenses stay on the PC. One folder \
                  per target, so the pendant can mark all files in a folder and copy them in one go. Files of the \
                  same name are overwritten, and the macOS \"._name\" files that a FAT or exFAT stick gets are removed."
)]
struct Args {
    /// Folder to write the files into (created if missing)
    dir: std::path::PathBuf,
}

fn main() -> std::io::Result<()> {
    let a = Args::parse();
    for (path, f) in write_files(&a.dir)?.iter().zip(FILES) {
        println!("{}  ->  {}", path.display(), f.robot_dir.unwrap_or("(keep on the PC)"));
    }
    Ok(())
}
