//! SPI Flash 存储器操作示例 (使用 SPI Bus 模式)
//!
//! 此示例演示如何使用 FTDI 芯片通过 SPI 接口与 Flash 存储器进行通信。
//! 使用 embedded-hal-bus 库的 RefCellDevice 来管理片选信号。
//!
//! SPI Flash 常见型号:
//! - W25Q32/64/128 系列 (Winbond)
//! - MX25L 系列 (Macronix)  
//! - AT25SF 系列 (Adesto)
//! - S25FL 系列 (Spansion)
//!
//! 硬件连接:
//! - SCK (Serial Clock): FTDI AD0 (Pin 0)
//! - MOSI (Master Out Slave In): FTDI AD1 (Pin 1)
//! - MISO (Master In Slave Out): FTDI AD2 (Pin 2)
//! - CS (Chip Select): FTDI AD3 (Pin 3) - 通过 GPIO 控制
//! - VCC: 3.3V 或 1.8V (根据 Flash 型号)
//! - GND: 接地
//!
//! 注意事项:
//! - 此示例会对整个 Flash 进行编程操作，会擦除原有数据!
//! - 在实际应用中请谨慎使用，确保备份重要数据
//!
//! 运行方式:
//! ```bash
//! RUST_LOG=info cargo run --example spibus_flash
//! ```

use std::sync::{Arc, Mutex};

use anyhow::anyhow;
use eh1::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use ftdi_tools::{
    Interface, Pin, gpio::FtdiOutputPin, list_all_device, mpsse::FtdiMpsse, spi::FtdiSpi,
};
use spi_flash::{Error, Flash, FlashAccess};

/// Flash 设备包装结构体
///
/// 将 SpiDevice trait 包装成 FlashAccess trait，以便与 spi-flash 库兼容
struct FlashDevice<T>(T);

/// 为 FlashDevice 实现 FlashAccess trait
///
/// 这个实现将 embedded-hal 的 SpiDevice 接口转换为 spi-flash 库所需的 FlashAccess 接口
impl<T: SpiDevice> FlashAccess for FlashDevice<T> {
    type Error = Error;

    /// 执行 SPI 数据交换操作
    ///
    /// # 参数
    /// * `data` - 要发送到 Flash 的数据
    ///
    /// # 返回值
    /// * `Ok(Vec<u8>)` - 从 Flash 接收到的数据
    /// * `Err(Error)` - SPI 通信错误
    fn exchange(&mut self, data: &[u8]) -> core::result::Result<Vec<u8>, Self::Error> {
        // 创建与发送数据相同长度的接收缓冲区
        let mut result = vec![0; data.len()];
        // 执行 SPI 全双工传输，同时发送和接收数据
        self.0
            .transfer(&mut result, data)
            .map_err(|_| Error::Access(anyhow!("SPI transfer failed")))?;
        Ok(result)
    }
}

/// 主函数 - SPI Flash 操作程序入口
///
/// 此函数演示完整的 SPI Flash 操作流程:
/// 1. 初始化 FTDI 设备和 SPI 接口
/// 2. 配置 GPIO 作为片选信号
/// 3. 读取 Flash 设备信息
/// 4. 执行全片编程操作
///
/// 返回值: anyhow::Result<()> - 成功或错误信息
fn main() -> anyhow::Result<()> {
    // 初始化日志系统以输出调试信息
    env_logger::init();

    // 获取系统中所有可用的 FTDI 设备
    let devices = list_all_device();
    // 验证至少存在一个可用的 FTDI 设备
    assert!(!devices.is_empty(), "Not found Ftdi devices");

    // 打开第一个 FTDI 设备的接口 A
    // 接口 A 通常具有最完整的 MPSSE 功能支持
    let mpsse = FtdiMpsse::open(&devices[0].usb_device, Interface::A)?;
    // 使用 Arc<Mutex<>> 包装以支持多线程安全访问
    let mtx = Arc::new(Mutex::new(mpsse));

    // 创建 SPI 控制器
    let spi = FtdiSpi::new(mtx.clone())?;

    // 创建 GPIO 输出引脚用于控制 Flash 的片选 (CS) 信号
    // Pin::Lower(3) 对应 FTDI AD3 引脚
    let cs = FtdiOutputPin::new(mtx.clone(), Pin::Lower(3))?;

    // 创建 SPI 设备实例，结合 SPI 总线和片选控制
    // RefCellDevice::new_no_delay 创建一个没有延时的 SPI 设备实例
    let mut flash_device = FlashDevice(ExclusiveDevice::new_no_delay(spi, cs)?);

    // 初始化 Flash 存储器接口
    let mut flash = Flash::new(&mut flash_device);

    // 读取并显示 Flash 设备的 JEDEC ID
    // JEDEC ID 通常包含 3 字节: 厂商 ID + 设备类型 + 容量
    let id = flash.read_id()?;
    println!("Flash JEDEC ID: {}", id);

    // 读取 Flash 设备的参数信息 (如容量、擦除块大小等)
    let param = flash.read_params()?.unwrap();
    println!("Flash parameters: {}", param);

    // 创建测试数据: 从 0 到 capacity-1 的字节序列
    // 警告: 这会生成与 Flash 容量相同大小的数据！
    let data: Vec<_> = (0..param.capacity_bytes()).map(|x| x as u8).collect();

    // 执行全片编程操作
    // 参数说明: 起始地址=0, 数据=data, 校验=true
    // 注意: 这个操作会擦除整个 Flash 并重新编程!
    flash.program_progress(0, &data, true)?;
    Ok(())
}
