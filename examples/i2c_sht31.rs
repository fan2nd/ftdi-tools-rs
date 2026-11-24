//! Example: i2c sht31

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use ftdi_tools::{i2c::FtdiI2c, list_all_device, mpsse::FtdiMpsse};
use sht31::prelude::*;

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();

    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;

    let mtx = Arc::new(Mutex::new(mpsse));

    let mut i2c = FtdiI2c::new(mtx)?;

    i2c.enable_fast(true);

    let addr_set = i2c.scan();

    println!("i2c detect:{:#x?}", addr_set);

    let mut sht = SHT31::single_shot(i2c, SingleShot::new())
        .with_accuracy(Accuracy::High)
        .with_unit(TemperatureUnit::Celsius)
        .with_address(DeviceAddr::AD0);

    sht.measure()?;
    std::thread::sleep(Duration::from_secs(1));

    if let Ok(reading) = sht.read() {
        println!("{reading:?}");
    }
    Ok(())
}
