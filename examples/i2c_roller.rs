//! Example: i2c roller

use eh1::i2c::I2c;
use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use std::sync::{Arc, Mutex};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();
    assert!(!devices.is_empty(), "No FTDI device found");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    let mtx = Arc::new(Mutex::new(mpsse));

    let mut i2c = FtdiI2c::new(mtx)?;

    let addr_set = i2c.scan();
    println!("Detected I2C devices: {:#x?}", addr_set);

    i2c.write(0x64, &[0x00, 0x01]).ok();
    i2c.write(0x64, &[0x01, 0x01]).ok();
    let a = 160000u32.to_le_bytes();
    i2c.write(0x64, &[0x40, a[0], a[1], a[2], a[3]]).ok();
    Ok(())
}
