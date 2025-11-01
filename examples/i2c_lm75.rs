//! I2C LM75 Temperature Sensor Example
//! Read LM75 temperature sensor data via I2C interface using FTDI chip
use anyhow::anyhow;
use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use lm75::Lm75;
use std::sync::{Arc, Mutex};

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

    // Initialize LM75 sensor
    let mut lm75 = Lm75::new(i2c, addr_set[0]);

    // Read temperature data
    let temp = lm75.read_temperature().map_err(|e| {
        if let lm75::Error::I2C(inner) = e {
            anyhow!(inner)
        } else {
            anyhow!("LM75 internal error")
        }
    })?;

    // Output temperature value
    println!("Temperature: {}°C", temp);
    Ok(())
}
