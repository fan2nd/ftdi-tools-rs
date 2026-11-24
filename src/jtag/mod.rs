//! JTAG modules and public exports.

mod hw_jtag;
mod jtag_detect;

pub use hw_jtag::FtdiJtag;
pub use jtag_detect::{JtagDetectTdi, JtagDetectTdo};
