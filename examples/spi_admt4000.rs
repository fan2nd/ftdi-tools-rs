//! Example: spi admt4000

use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use admt4000::Admt4000;
use ftdi_tools::{Interface, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpiDevice};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    mpsse.set_frequency(10_000_000).unwrap();

    let mtx = Arc::new(Mutex::new(mpsse));
    let spi_device = FtdiSpiDevice::new(mtx)?;

    let mut admt4000 = Admt4000::new(spi_device);
    println!("fault: {:#x}", admt4000.read_fault());
    admt4000.clear_fault();
    loop {
        println!("{:?}", admt4000.read_abs_angle());
        thread::sleep(Duration::from_secs(1));
    }
}
