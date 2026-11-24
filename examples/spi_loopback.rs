//! Example: spi loopback

use std::sync::{Arc, Mutex};

use eh1::spi::{Operation, SpiDevice};
use ftdi_tools::{Interface, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpiDevice};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();

    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;

    let mtx = Arc::new(Mutex::new(mpsse));

    let mut spidevice = FtdiSpiDevice::new(mtx)?;

    let max = 6000000;
    let mid = 600000;
    let data: Vec<_> = (0..max).map(|x| (x * 7 + 3) as u8).collect();
    let mut read_back = vec![0; max];
    let (read_back1, read_back2) = read_back.split_at_mut(mid);
    spidevice.transaction(&mut [
        Operation::Transfer(read_back1, &data[0..mid]),
        Operation::Transfer(read_back2, &data[mid..max]),
    ])?;

    read_back
        .into_iter()
        .zip(data)
        .for_each(|x| assert_eq!(x.0, x.1));
    Ok(())
}
