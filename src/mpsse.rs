//! High-level MPSSE controller and FTDI pin state management.

use futures_lite::future::block_on;
use nusb::MaybeFuture;
use std::sync::{Arc, Mutex};

use crate::{ChipType, FtdiError, Interface, Pin, ftdaye::FtdiContext, mpsse_cmd::MpsseCmdBuilder};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinUsage {
    Output,
    Input,
    I2c,
    Spi,
    Jtag,
    Swd,
}

#[derive(Debug, Default)]
pub(crate) struct GpioByte {
    pub(crate) direction: u8,
    pub(crate) value: u8,
    pins: [Option<PinUsage>; 8],
}

pub struct FtdiMpsse {
    ft: FtdiContext,
    interface: Interface,
    pub(crate) chip_type: ChipType,
    pub(crate) lower: GpioByte,
    pub(crate) upper: GpioByte,
}

impl FtdiMpsse {
    pub fn open(usb_device: &nusb::DeviceInfo, interface: Interface) -> Result<Self, FtdiError> {
        let chip_type = ChipType::from_device(usb_device)?;
        if !chip_type.interface_list().contains(&interface) {
            return Err(FtdiError::OpenFailed(format!(
                "{chip_type:?} do not support Interface::{interface:?}"
            )));
        }

        let handle = usb_device.open().wait().map_err(std::io::Error::from)?;
        let handle = handle
            .detach_and_claim_interface(interface.interface_number())
            .wait()
            .map_err(std::io::Error::from)?;

        let this = Self {
            ft: FtdiContext::new(handle, interface).into_mpsse(0)?,
            interface,
            chip_type,
            lower: Default::default(),
            upper: Default::default(),
        };

        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(0, 0)
            .set_gpio_upper(0, 0)
            .enable_loopback(false);
        if chip_type == ChipType::FT2232D {
            cmd.set_clock(0, None);
        } else {
            cmd.enable_3phase_data_clocking(false)
                .enable_adaptive_clocking(false)
                .set_clock(0, Some(false));
        }
        this.exec(cmd)?;

        Ok(this)
    }

    pub fn set_frequency(&self, frequency_hz: usize) -> Result<usize, FtdiError> {
        let (max_frequency, clk_div_by5) = self.chip_type.max_frequecny();
        let min_frequency = max_frequency / (u16::MAX as usize + 1) + 1;

        let divisor = if frequency_hz > max_frequency {
            log::warn!("frequency has out of range[{min_frequency}-{max_frequency}Hz]");
            log::warn!("frequency set to {max_frequency}Hz]");
            1
        } else if frequency_hz < min_frequency {
            log::warn!("frequency has out of range[{min_frequency}-{max_frequency}Hz]");
            log::warn!("frequency set to {min_frequency}Hz]");
            u16::MAX as usize + 1
        } else if max_frequency % frequency_hz != 0 {
            max_frequency / frequency_hz + 1
        } else {
            max_frequency / frequency_hz
        };

        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_clock((divisor - 1) as u16, clk_div_by5);
        self.exec(cmd)?;
        log::info!("Frequency set to {}Hz", max_frequency / divisor);
        Ok(max_frequency / divisor)
    }

    // Keep synchronous GPIO users and async bus users on the same transaction lock.
    // A cancelled call that is still queued must not touch pins after they are reused.
    pub(crate) async fn with_controller<T: Send + 'static>(
        controller: Arc<Mutex<Self>>,
        operation: impl FnOnce(&Self) -> T + Send + 'static,
    ) -> T {
        let active = Arc::new(());
        let pending = Arc::downgrade(&active);
        let result = blocking::unblock(move || {
            let lock = controller.lock().unwrap();
            pending.upgrade().map(|_| operation(&lock))
        })
        .await;
        drop(active);
        result.expect("the awaiting transaction is still active")
    }

    async fn exec_async(&self, cmd: impl Into<MpsseCmdBuilder>) -> Result<Vec<u8>, FtdiError> {
        let cmd = cmd.into();
        let (cmd, mut response) = cmd.destruct();
        self.ft.async_write_read(cmd, &mut response).await?;
        Ok(response)
    }

    pub(crate) fn exec(&self, cmd: impl Into<MpsseCmdBuilder>) -> Result<Vec<u8>, FtdiError> {
        block_on(self.exec_async(cmd))
    }

    pub(crate) fn alloc_pin(&mut self, pin: Pin, usage: PinUsage) -> Result<(), FtdiError> {
        if !self.chip_type.mpsse_list().contains(&self.interface)
            && (usage != PinUsage::Input && usage != PinUsage::Output)
        {
            return Err(FtdiError::PinFault(format!(
                "{:?} Interface::{:?} can not be used for {usage:?}",
                self.chip_type, self.interface
            )));
        };
        let (byte, idx) = match pin {
            Pin::Lower(idx) => {
                if idx >= 8 {
                    return Err(FtdiError::PinFault(format!(
                        "{:?} Interface::{:?} do not has {pin:?}",
                        self.chip_type, self.interface
                    )));
                };
                (&mut self.lower, idx)
            }
            Pin::Upper(idx) => {
                if idx >= self.chip_type.upper_pins() {
                    return Err(FtdiError::PinFault(format!(
                        "{:?} Interface::{:?} do not has {pin:?}",
                        self.chip_type, self.interface
                    )));
                }
                (&mut self.upper, idx)
            }
        };
        if let Some(current) = byte.pins[idx] {
            return Err(FtdiError::PinFault(format!(
                "Unable to allocate pin {pin:?} for {usage:?}, pin is already allocated for {current:?}"
            )));
        } else {
            log::trace!("pin {:?} has been alloced for {:?}", pin, usage);
            byte.pins[idx] = Some(usage)
        }
        Ok(())
    }

    pub(crate) fn free_pin(&mut self, pin: Pin) {
        log::trace!("pin {:?} has been released", pin);
        match pin {
            Pin::Lower(idx) => {
                if idx >= 8 {
                    log::error!("Skip releasing invalid pin index {idx} on lower bank");
                    return;
                }
                self.lower.pins[idx] = None;
                self.lower.value &= !pin.mask();
                self.lower.direction &= !pin.mask();
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_lower(self.lower.value, self.lower.direction);
                if let Err(e) = self.exec(cmd) {
                    log::warn!("Failed to flush lower pin state while releasing {pin:?}: {e}");
                }
            }
            Pin::Upper(idx) => {
                if idx >= 8 {
                    log::error!("Skip releasing invalid pin index {idx} on upper bank");
                    return;
                }
                self.upper.pins[idx] = None;
                self.upper.value &= !pin.mask();
                self.upper.direction &= !pin.mask();
                let mut cmd = MpsseCmdBuilder::new();
                cmd.set_gpio_upper(self.upper.value, self.upper.direction);
                if let Err(e) = self.exec(cmd) {
                    log::warn!("Failed to flush upper pin state while releasing {pin:?}: {e}");
                }
            }
        };
    }
}
