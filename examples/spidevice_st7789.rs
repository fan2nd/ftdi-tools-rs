//! Example: spidevice st7789

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use anyhow::anyhow;
use eh1::digital::OutputPin;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use ftdi_tools::{
    Interface, Pin, delay::Delay, gpio::FtdiOutputPin, list_all_device, mpsse::FtdiMpsse,
    spi::FtdiSpiDevice,
};
use mipidsi::{
    Builder, TestImage,
    interface::SpiInterface,
    models::ST7789,
    options::{ColorInversion, ColorOrder},
};

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let devices = list_all_device();

    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;

    let mtx = Arc::new(Mutex::new(mpsse));

    let spidevice = FtdiSpiDevice::new(mtx.clone())?;
    let rst = FtdiOutputPin::new(mtx.clone(), Pin::Lower(4))?;
    let dc = FtdiOutputPin::new(mtx.clone(), Pin::Lower(5))?;
    let mut blk = FtdiOutputPin::new(mtx.clone(), Pin::Lower(6))?;
    blk.set_high()?;
    let buffer = &mut [0; 240 * 280 * 2];
    let interface = SpiInterface::new(spidevice, dc, buffer);

    let mut display = Builder::new(ST7789, interface)
        .reset_pin(rst)
        .display_size(240, 280)
        .display_offset(0, 20)
        .invert_colors(ColorInversion::Inverted)
        .color_order(ColorOrder::Rgb)
        .init(&mut Delay)
        .map_err(|x| anyhow!("{x:?}"))?;
    display.clear(RgbColor::RED).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    display.clear(RgbColor::GREEN).unwrap();
    std::thread::sleep(Duration::from_millis(500));
    display.clear(RgbColor::BLUE).unwrap();

    let style = PrimitiveStyleBuilder::new()
        .fill_color(RgbColor::BLACK)
        .build();
    Rectangle::new(Point { x: 75, y: 90 }, Size::new(100, 100))
        .into_styled(style)
        .draw(&mut display)
        .unwrap();
    std::thread::sleep(Duration::from_millis(500));
    TestImage::new().draw(&mut display).unwrap();

    let style = MonoTextStyle::new(&FONT_10X20, RgbColor::BLACK);
    Text::new("Hello Rust!", Point { x: 26, y: 120 }, style)
        .draw(&mut display)
        .unwrap();
    Ok(())
}
