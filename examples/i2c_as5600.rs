//! I2C LM75 Temperature Sensor Example
//! Read LM75 temperature sensor data via I2C interface using FTDI chip
use as5600::As5600;
use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

fn main() -> anyhow::Result<()> {
    // Initialize logging system
    env_logger::init();

    // Scan and connect to FTDI device
    let devices = list_all_device();
    assert!(!devices.is_empty(), "No FTDI device found");

    // Initialize MPSSE mode
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;
    let mtx = Arc::new(Mutex::new(mpsse));

    // Create I2C controller
    let mut i2c = FtdiI2c::new(mtx)?;

    // Scan I2C devices
    let addr_set = i2c.scan();
    println!("Detected I2C devices: {:#x?}", addr_set);

    let mut as5600 = As5600::new(i2c);

    loop {
        let angle = as5600.raw_angle().unwrap();
        println!("angle: {}", angle);
        thread::sleep(Duration::from_secs(1));
    }
}
