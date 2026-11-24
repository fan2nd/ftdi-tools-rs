//! Example: i2c max30102

use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use max3010x::Max3010x;
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

    let mut max30102 = Max3010x::new_max30102(i2c);

    max30102.reset().unwrap();
    let id = max30102.get_revision_id().unwrap();
    println!("MAX30102 revision ID: {id:#04x}");

    Ok(())
}
