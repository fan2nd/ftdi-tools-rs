//! Example: i2c si5351

use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use si5351a_adafruit::{PLL, Si5351};
use std::sync::{Arc, Mutex};

fn main() {
    env_logger::init();

    let devices = list_all_device();
    assert!(!devices.is_empty(), "No FTDI device found");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0]).unwrap();
    let mtx = Arc::new(Mutex::new(mpsse));

    let mut i2c = FtdiI2c::new(mtx).unwrap();

    let addr_set = i2c.scan();
    println!("Detected I2C devices: {:#x?}", addr_set);

    let mut si5351 = Si5351::new();
    si5351.begin(i2c).unwrap();
    si5351.set_freq(0, PLL::A, 50000000).unwrap();
    si5351.enable_outputs(true).unwrap();
}
