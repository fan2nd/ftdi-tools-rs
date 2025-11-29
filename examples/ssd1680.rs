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
    // 初始化日志输出系统
    env_logger::init();

    // 扫描并获取所有连接的 FTDI 设备
    let devices = list_all_device();
    // 验证系统中存在可用的 FTDI 设备
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    // 打开第一个 FTDI 设备的接口 A
    // 接口 A 通常是主接口，支持全部 MPSSE 功能
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    mpsse.set_frequency(9_000_000)?;
    // 使用线程安全的 Arc<Mutex<>> 包装 MPSSE 控制器
    let mtx = Arc::new(Mutex::new(mpsse));

    // 创建 FtdiSpiDevice 实例
    // 这个设备封装了 SPI 总线和片选控制，提供了完整的 SpiDevice 实现
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
