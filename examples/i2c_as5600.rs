//! Example: i2c as5600

use as5600::As5600;
use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();
    assert!(!devices.is_empty(), "No FTDI device found");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    let mtx = Arc::new(Mutex::new(mpsse));

    let mut i2c = FtdiI2c::new(mtx)?;

    let addr_set = i2c.scan();
    println!("Detected I2C devices: {:#x?}", addr_set);

    let mut as5600 = As5600::new(i2c);

    loop {
        let angle = as5600.raw_angle().unwrap();
        println!("angle: {}", angle);
        thread::sleep(Duration::from_secs(1));
    }
}
