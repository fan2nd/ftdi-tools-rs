//! Example: jtag scan chains

use std::{
    sync::{Arc, Mutex},
    time::Instant,
};

use ftdi_tools::{jtag::FtdiJtag, list_all_device, mpsse::FtdiMpsse};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let now = Instant::now();

    let devices = list_all_device();

    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;

    let mtx = Arc::new(Mutex::new(mpsse));

    let mut jtag = FtdiJtag::new(mtx)?;

    let ids = jtag.scan_with(true)?;

    println!("Scan Result:{ids:#x?}");

    println!("Finish Scan Using {:?}", now.elapsed());
    Ok(())
}
