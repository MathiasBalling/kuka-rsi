//! The robot-side files (RSI contexts, Ethernet configs, KRL programs), bundled into the
//! library so they ship with the crate and the Python package. `controller/README.md` and
//! `controller/BRINGUP.md` in the repository describe how to install them.

use std::io;
use std::path::{Path, PathBuf};

/// Where RSI contexts and Ethernet configs go on a KR C5.
pub const SENSOR_INTERFACE_DIR: &str = r"C:\KRC\ROBOTER\Config\User\Common\SensorInterface\";
/// Where the KRL programs go on a KR C5.
pub const PROGRAM_DIR: &str = r"KRC:\R1\Program\";

/// One bundled file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerFile {
    pub name: &'static str,
    pub contents: &'static str,
    /// Target folder on the controller; `None` for files that stay on the PC (the license).
    pub robot_dir: Option<&'static str>,
}

const fn file(name: &'static str, contents: &'static str, robot_dir: Option<&'static str>) -> ControllerFile {
    ControllerFile { name, contents, robot_dir }
}

/// The Ethernet config `kuka_rsi_ethernet.xml` is also the driver's telegram definition: pass its
/// path to `DriverConfig::from_ethernet_xml` (after editing `IP_NUMBER`).
pub const FILES: &[ControllerFile] = &[
    file("kuka_rsi_ethernet.xml", include_str!("../controller/kuka_rsi_ethernet.xml"), Some(SENSOR_INTERFACE_DIR)),
    file("kuka_rsi_monitor.xml", include_str!("../controller/kuka_rsi_monitor.xml"), Some(SENSOR_INTERFACE_DIR)),
    file("kuka_rsi.rsix", include_str!("../controller/kuka_rsi.rsix"), Some(SENSOR_INTERFACE_DIR)),
    file("kuka_rsi_monitor.rsix", include_str!("../controller/kuka_rsi_monitor.rsix"), Some(SENSOR_INTERFACE_DIR)),
    file("kuka_rsi.src", include_str!("../controller/kuka_rsi.src"), Some(PROGRAM_DIR)),
    file("kuka_rsi_monitor.src", include_str!("../controller/kuka_rsi_monitor.src"), Some(PROGRAM_DIR)),
    file("LICENSE-FZI", include_str!("../controller/LICENSE-FZI"), None),
];

impl ControllerFile {
    /// The bytes to copy onto the controller: the KRL programs with CRLF line endings (as the
    /// controller's own editor writes them), every other file as shipped.
    pub fn robot_bytes(&self) -> Vec<u8> {
        if self.robot_dir == Some(PROGRAM_DIR) {
            self.contents.replace("\r\n", "\n").replace('\n', "\r\n").into_bytes()
        } else {
            self.contents.as_bytes().to_vec()
        }
    }
}

/// Write every bundled file ([`ControllerFile::robot_bytes`]) into `dir` (created if missing),
/// overwriting files of the same name. Returns the written paths in the order of [`FILES`].
///
/// On a FAT or exFAT USB stick, macOS stores extended attributes in an AppleDouble file
/// `._<name>` next to every file and folder it writes; the controller would show them as broken
/// programs and configs. They are removed for every file written, for `dir` and for the folders
/// created above it. No other files are touched.
pub fn write_files(dir: &Path) -> io::Result<Vec<PathBuf>> {
    let created: Vec<PathBuf> =
        dir.ancestors().take_while(|d| !d.as_os_str().is_empty() && !d.exists()).map(Path::to_path_buf).collect();
    std::fs::create_dir_all(dir)?;
    let paths = FILES
        .iter()
        .map(|f| {
            let path = dir.join(f.name);
            std::fs::write(&path, f.robot_bytes())?;
            Ok(path)
        })
        .collect::<io::Result<Vec<_>>>()?;
    for p in paths.iter().map(PathBuf::as_path).chain([dir]).chain(created.iter().map(PathBuf::as_path)) {
        remove_appledouble(p)?;
    }
    Ok(paths)
}

/// Remove the macOS AppleDouble file `._<name>` that sits next to `path`, if there is one.
fn remove_appledouble(path: &Path) -> io::Result<()> {
    let (Some(parent), Some(name)) = (path.parent(), path.file_name()) else { return Ok(()) };
    let mut double = std::ffi::OsString::from("._");
    double.push(name);
    match std::fs::remove_file(parent.join(double)) {
        Err(e) if e.kind() != io::ErrorKind::NotFound => Err(e),
        _ => Ok(()),
    }
}
