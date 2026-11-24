#![forbid(unsafe_code)]

#[cfg(not(any(target_os = "windows", target_os = "linux", target_os = "macos")))]
compile_error!("ftdi-tools supports only desktop OS targets: windows, linux, macos");

pub mod delay;
pub mod eeprom;
mod ftdaye;
pub mod gpio;
pub mod i2c;
pub mod jtag;
mod list;
pub use list::{FtdiDeviceInfo, list_all_device, list_devices, list_devices_with_ids};
pub mod mpsse;
mod mpsse_cmd;
pub mod spi;
pub mod swd;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipType {
    Am,
    Bm,
    FT2232D,
    R,
    FT2232H,
    FT4232H,
    FT232H,
    FT230X,
    Unknown,
}
impl ChipType {
    fn from_device(info: &nusb::DeviceInfo) -> Result<Self, FtdiError> {
        Self::from_version(info.device_version(), info.serial_number())
    }

    fn from_version(version: u16, serial: Option<&str>) -> Result<Self, FtdiError> {
        let chip = match version {
            0x0500 => Self::FT2232D,
            0x0700 => Self::FT2232H,
            0x0800 => Self::FT4232H,
            0x0900 => Self::FT232H,
            0x0400 => Self::Bm,
            0x0200 if serial.unwrap_or("").is_empty() => Self::Bm,
            0x0200 => Self::Am,
            0x0600 => Self::R,
            0x1000 => Self::FT230X,
            _ => Self::Unknown,
        };
        if chip.interface_list().is_empty() {
            return Err(FtdiError::UnsupportedChip(chip));
        }
        Ok(chip)
    }

    pub(crate) const fn interface_list(self) -> &'static [Interface] {
        match self {
            ChipType::FT232H => &[Interface::A],
            ChipType::FT2232H | ChipType::FT2232D => &[Interface::A, Interface::B],
            ChipType::FT4232H => &[Interface::A, Interface::B, Interface::C, Interface::D],
            _ => &[],
        }
    }
    pub(crate) const fn mpsse_list(self) -> &'static [Interface] {
        match self {
            ChipType::FT232H | ChipType::FT2232D => &[Interface::A],
            ChipType::FT2232H | ChipType::FT4232H => &[Interface::A, Interface::B],
            _ => &[],
        }
    }
    pub(crate) const fn upper_pins(self) -> usize {
        match self {
            ChipType::FT232H | ChipType::FT2232H => 8,
            ChipType::FT2232D => 4,
            ChipType::FT4232H => 0,
            _ => 0,
        }
    }
    pub(crate) const fn max_frequecny(self) -> (usize, Option<bool>) {
        match self {
            ChipType::FT2232D => (6_000_000, None),
            ChipType::FT232H | ChipType::FT2232H | ChipType::FT4232H => (30_000_000, Some(false)),
            _ => (0, None),
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Interface {
    A = 1,
    B = 2,
    C = 3,
    D = 4,
}

impl Interface {
    pub(crate) const fn read_ep(self) -> u8 {
        match self {
            Interface::A => 0x81,
            Interface::B => 0x83,
            Interface::C => 0x85,
            Interface::D => 0x87,
        }
    }

    pub(crate) const fn write_ep(self) -> u8 {
        match self {
            Interface::A => 0x02,
            Interface::B => 0x04,
            Interface::C => 0x06,
            Interface::D => 0x08,
        }
    }

    pub(crate) const fn index(self) -> u16 {
        self as u16
    }

    pub(crate) const fn interface_number(self) -> u8 {
        (self as u8) - 1
    }
}
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Pin {
    Lower(usize),
    Upper(usize),
}
impl Pin {
    pub(crate) const fn mask(self) -> u8 {
        match self {
            Pin::Lower(idx) => 1 << idx,
            Pin::Upper(idx) => 1 << idx,
        }
    }
}
#[derive(Debug, thiserror::Error)]
pub enum FtdiError {
    #[error("A USB transport error occurred.")]
    Usb(#[from] std::io::Error),

    #[error("Open failed: {0}")]
    OpenFailed(String),

    #[error("Unsupported chip type: {0:?}")]
    UnsupportedChip(ChipType),

    #[error("Bad Mpsse Command: {0:#x}")]
    BadMpsseCommand(u8),

    #[error("Pin fault: {0}")]
    PinFault(String),

    #[error("{0}")]
    Other(&'static str),
}
