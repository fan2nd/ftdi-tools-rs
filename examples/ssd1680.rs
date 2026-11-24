//! Example: ssd1680

use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use eh1::{
    digital::{InputPin, OutputPin},
    spi::SpiDevice,
};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::*, primitives::Rectangle};
use embedded_hal_bus::spi::ExclusiveDevice;
use ftdi_tools::{
    Interface, Pin,
    gpio::{FtdiInputPin, FtdiOutputPin},
    list_all_device,
    mpsse::FtdiMpsse,
    spi::FtdiSpi,
};
use ssd1680::{Ssd1680, Ssd1680Buffer};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    mpsse.set_frequency(9_000_000)?;

    let mtx = Arc::new(Mutex::new(mpsse));

    let spi = FtdiSpi::new(mtx.clone())?;
    let mut rst = FtdiOutputPin::new(mtx.clone(), Pin::Lower(4))?;
    let dc = FtdiOutputPin::new(mtx.clone(), Pin::Lower(5))?;
    let cs = FtdiOutputPin::new(mtx.clone(), Pin::Lower(6))?;
    let mut busy = FtdiInputPin::new(mtx.clone(), Pin::Lower(7))?;

    let mut ssd1680 = Ssd1680::new(ExclusiveDevice::new_no_delay(spi, cs)?, dc);

    rst.set_low().unwrap();
    rst.set_high().unwrap();
    unsafe {
        ssd1680.assert_hard_reset();
    }
    wait_not_busy(&mut ssd1680, &mut busy);
    ssd1680.soft_reset().unwrap();
    wait_not_busy(&mut ssd1680, &mut busy);
    ssd1680.init_fast_seq1().unwrap();
    wait_not_busy(&mut ssd1680, &mut busy);
    ssd1680.init_fast_seq2().unwrap();
    wait_not_busy(&mut ssd1680, &mut busy);

    let mut buffer = Ssd1680Buffer::<152, 152, 2888>::new();

    for i in 0..10 {
        buffer
            .fill_solid(
                &Rectangle {
                    top_left: Point {
                        x: 10 * i,
                        y: 10 * i,
                    },
                    size: Size {
                        width: 10 * i as u32,
                        height: 10 * i as u32,
                    },
                },
                BinaryColor::On,
            )
            .unwrap();
        ssd1680.part_update(&buffer.0).unwrap();
        wait_not_busy(&mut ssd1680, &mut busy);
        thread::sleep(Duration::from_secs(1));
    }
    Ok(())
}

fn wait_not_busy<SPI: SpiDevice, DC: OutputPin, BUSY: InputPin>(
    ssd1680: &mut Ssd1680<SPI, DC>,
    busy: &mut BUSY,
) {
    while busy.is_high().unwrap() {}
    unsafe { ssd1680.assert_not_busy() }
}
