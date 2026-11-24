//! SPI bus and SPI device implementations built on FTDI MPSSE.

use crate::{
    FtdiError, Pin,
    gpio::UsedPin,
    mpsse::{FtdiMpsse, PinUsage},
    mpsse_cmd::MpsseCmdBuilder,
};
use eh1::spi::{Error, ErrorKind, ErrorType, MODE_0, MODE_2, Mode, Operation, SpiBus, SpiDevice};
use std::sync::{Arc, Mutex};

const SCK_MASK: u8 = Pin::Lower(0).mask();
const MOSI_MASK: u8 = Pin::Lower(1).mask();
#[allow(unused)]
const MISO_MASK: u8 = Pin::Lower(2).mask();
const CS_MASK: u8 = Pin::Lower(3).mask();

#[derive(Debug, thiserror::Error)]
pub enum FtdiSpiError {
    #[error(transparent)]
    FtdiInner(#[from] FtdiError),
    #[error("embedded-hal::spi::SpiBus {0} is not supported.")]
    NotSupported(&'static str),
}
impl Error for FtdiSpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct FtdiSpi {
    _pins: [UsedPin; 3],

    mtx: Arc<Mutex<FtdiMpsse>>,

    tck_init_value: bool,

    is_lsb: bool,
}

impl FtdiSpi {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUsage::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };

        let mut lock = mtx.lock().unwrap();

        lock.lower.direction |= SCK_MASK | MOSI_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;

        Ok(this)
    }

    pub fn set_mode(&mut self, mode: Mode, is_lsb: bool) -> Result<(), FtdiSpiError> {
        let mut lock = self.mtx.lock().unwrap();

        match mode {
            MODE_0 => {
                lock.lower.value &= !SCK_MASK;
                self.tck_init_value = false;
            }
            MODE_2 => {
                lock.lower.value |= SCK_MASK;
                self.tck_init_value = true;
            }
            _ => {
                return Err(FtdiSpiError::NotSupported("MODE_1&MODE_3"));
            }
        }
        self.is_lsb = is_lsb;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;
        Ok(())
    }
}

impl ErrorType for FtdiSpi {
    type Error = FtdiSpiError;
}

impl SpiBus<u8> for FtdiSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let lock = self.mtx.lock().unwrap();
        let response = lock.exec(cmd)?;
        words.copy_from_slice(&response);

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_out(self.tck_init_value, self.is_lsb, words);

        let lock = self.mtx.lock().unwrap();
        lock.exec(cmd)?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes(self.tck_init_value, self.is_lsb, words);

        let lock = self.mtx.lock().unwrap();

        let response = lock.exec(cmd)?;
        words.copy_from_slice(&response);

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.transfer_bytes(self.tck_init_value, self.is_lsb, read.len(), write);

        let lock = self.mtx.lock().unwrap();
        let response = lock.exec(cmd)?;
        read.copy_from_slice(&response);

        Ok(())
    }
}

pub struct FtdiSpiHalfduplex {
    _pins: [UsedPin; 3],

    mtx: Arc<Mutex<FtdiMpsse>>,

    tck_init_value: bool,

    is_lsb: bool,
}

impl FtdiSpiHalfduplex {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSpiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUsage::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };

        let mut lock = mtx.lock().unwrap();

        lock.lower.direction |= SCK_MASK | MOSI_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;

        Ok(this)
    }

    pub fn set_mode(&mut self, mode: Mode, is_lsb: bool) -> Result<(), FtdiSpiError> {
        let mut lock = self.mtx.lock().unwrap();

        match mode {
            MODE_0 => {
                lock.lower.value &= !SCK_MASK;
                self.tck_init_value = false;
            }
            MODE_2 => {
                lock.lower.value |= SCK_MASK;
                self.tck_init_value = true;
            }
            _ => {
                return Err(FtdiSpiError::NotSupported("MODE_1&MODE_3"));
            }
        }
        self.is_lsb = is_lsb;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;
        Ok(())
    }
}

impl ErrorType for FtdiSpiHalfduplex {
    type Error = FtdiSpiError;
}

impl SpiBus for FtdiSpiHalfduplex {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction & (!MOSI_MASK));
        cmd.shift_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let response = lock.exec(cmd)?;
        words.copy_from_slice(&response);

        Ok(())
    }
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        cmd.shift_bytes_out(self.tck_init_value, self.is_lsb, words);

        lock.exec(cmd)?;

        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer"))
    }
    fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer_in_place"))
    }
}

pub struct FtdiSpiTx {
    _pins: [UsedPin; 2],

    mtx: Arc<Mutex<FtdiMpsse>>,

    tck_init_value: bool,

    is_lsb: bool,
}

impl FtdiSpiTx {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSpiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };

        let mut lock = mtx.lock().unwrap();

        lock.lower.direction |= SCK_MASK | MOSI_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;

        Ok(this)
    }

    pub fn set_mode(&mut self, mode: Mode, is_lsb: bool) -> Result<(), FtdiSpiError> {
        let mut lock = self.mtx.lock().unwrap();

        match mode {
            MODE_0 => {
                lock.lower.value &= !SCK_MASK;
                self.tck_init_value = false;
            }
            MODE_2 => {
                lock.lower.value |= SCK_MASK;
                self.tck_init_value = true;
            }
            _ => {
                return Err(FtdiSpiError::NotSupported("MODE_1&MODE_3"));
            }
        }
        self.is_lsb = is_lsb;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;
        Ok(())
    }
}

impl ErrorType for FtdiSpiTx {
    type Error = FtdiSpiError;
}

impl SpiBus for FtdiSpiTx {
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        cmd.shift_bytes_out(self.tck_init_value, self.is_lsb, words);

        lock.exec(cmd)?;

        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("read"))
    }
    fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer"))
    }
    fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer_in_place"))
    }
}
pub struct FtdiSpiDevice {
    _pins: [UsedPin; 4],

    mtx: Arc<Mutex<FtdiMpsse>>,

    tck_init_value: bool,

    is_lsb: bool,
}

impl FtdiSpiDevice {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSpiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUsage::Spi)?,
                UsedPin::new(mtx.clone(), Pin::Lower(3), PinUsage::Spi)?,
            ],
            mtx: mtx.clone(),
            tck_init_value: false,
            is_lsb: false,
        };
        let mut lock = mtx.lock().unwrap();

        lock.lower.direction |= SCK_MASK | MOSI_MASK | CS_MASK;
        lock.lower.value |= CS_MASK;
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
        lock.exec(cmd)?;

        Ok(this)
    }
}

impl ErrorType for FtdiSpiDevice {
    type Error = FtdiSpiError;
}

// Each batch ends at a requested delay or at the end of the transaction.
struct SpiBatch {
    cmd: MpsseCmdBuilder,
    delay_ns: Option<u32>,
}

impl FtdiSpiDevice {
    fn batches(clock: bool, lsb: bool, operations: &[Operation<'_, u8>]) -> Vec<SpiBatch> {
        let mut batches = Vec::new();
        let mut cmd = MpsseCmdBuilder::new();
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    cmd.shift_bytes_in(clock, lsb, buf.len());
                }
                Operation::Write(buf) => {
                    cmd.shift_bytes_out(clock, lsb, buf);
                }
                Operation::Transfer(read, write) => {
                    cmd.transfer_bytes(clock, lsb, read.len(), write);
                }
                Operation::TransferInPlace(buf) => {
                    cmd.shift_bytes(clock, lsb, buf);
                }
                Operation::DelayNs(0) => {}
                Operation::DelayNs(ns) => batches.push(SpiBatch {
                    cmd: std::mem::take(&mut cmd),
                    delay_ns: Some(*ns),
                }),
            }
        }
        batches.push(SpiBatch {
            cmd,
            delay_ns: None,
        });
        batches
    }

    fn run_batches(lock: &FtdiMpsse, batches: Vec<SpiBatch>) -> Result<Vec<u8>, FtdiError> {
        let mut response = Vec::new();
        let mut cmd = MpsseCmdBuilder::new();
        cmd.set_gpio_lower(lock.lower.value & !CS_MASK, lock.lower.direction);
        for batch in batches {
            cmd.append(batch.cmd);
            if batch.delay_ns.is_some() {
                // A GPIO read confirms the MPSSE has executed preceding clocks.
                cmd.gpio_lower();
            } else {
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            let mut data = match lock.exec(std::mem::take(&mut cmd)) {
                Ok(data) => data,
                Err(error) => {
                    cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
                    let _ = lock.exec(cmd);
                    return Err(error);
                }
            };
            if let Some(ns) = batch.delay_ns {
                data.pop();
                std::thread::sleep(std::time::Duration::from_nanos(ns.into()));
            }
            response.append(&mut data);
        }
        Ok(response)
    }

    fn copy_response(operations: &mut [Operation<'_, u8>], mut response: &[u8]) {
        for op in operations {
            let buf = match op {
                Operation::Read(buf)
                | Operation::Transfer(buf, _)
                | Operation::TransferInPlace(buf) => buf,
                _ => continue,
            };
            let (data, rest) = response.split_at(buf.len());
            buf.copy_from_slice(data);
            response = rest;
        }
    }
}

impl SpiDevice<u8> for FtdiSpiDevice {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
        let batches = Self::batches(self.tck_init_value, self.is_lsb, operations);
        let response = Self::run_batches(&self.mtx.lock().unwrap(), batches)?;
        Self::copy_response(operations, &response);
        Ok(())
    }
}

impl embedded_hal_async::spi::SpiBus<u8> for FtdiSpi {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let response =
            FtdiMpsse::with_controller(self.mtx.clone(), move |lock| lock.exec(cmd)).await?;
        words.copy_from_slice(&response);

        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_out(self.tck_init_value, self.is_lsb, words);

        FtdiMpsse::with_controller(self.mtx.clone(), move |lock| lock.exec(cmd)).await?;

        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.transfer_bytes(self.tck_init_value, self.is_lsb, read.len(), write);

        let response =
            FtdiMpsse::with_controller(self.mtx.clone(), move |lock| lock.exec(cmd)).await?;
        read.copy_from_slice(&response);

        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes(self.tck_init_value, self.is_lsb, words);

        let response =
            FtdiMpsse::with_controller(self.mtx.clone(), move |lock| lock.exec(cmd)).await?;
        words.copy_from_slice(&response);

        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal_async::spi::SpiBus for FtdiSpiHalfduplex {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_in(self.tck_init_value, self.is_lsb, words.len());

        let response = FtdiMpsse::with_controller(self.mtx.clone(), move |lock| {
            let mut setup = MpsseCmdBuilder::new();
            setup.set_gpio_lower(lock.lower.value, lock.lower.direction & (!MOSI_MASK));
            setup.append(cmd);
            lock.exec(setup)
        })
        .await?;
        words.copy_from_slice(&response);

        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_out(self.tck_init_value, self.is_lsb, words);

        FtdiMpsse::with_controller(self.mtx.clone(), move |lock| {
            let mut setup = MpsseCmdBuilder::new();
            setup.set_gpio_lower(lock.lower.value, lock.lower.direction);
            setup.append(cmd);
            lock.exec(setup)
        })
        .await?;

        Ok(())
    }

    async fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer"))
    }

    async fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer_in_place"))
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal_async::spi::SpiBus for FtdiSpiTx {
    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut cmd = MpsseCmdBuilder::new();
        cmd.shift_bytes_out(self.tck_init_value, self.is_lsb, words);

        FtdiMpsse::with_controller(self.mtx.clone(), move |lock| {
            let mut setup = MpsseCmdBuilder::new();
            setup.set_gpio_lower(lock.lower.value, lock.lower.direction);
            setup.append(cmd);
            lock.exec(setup)
        })
        .await?;

        Ok(())
    }

    async fn read(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("read"))
    }

    async fn transfer(&mut self, _read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer"))
    }

    async fn transfer_in_place(&mut self, _words: &mut [u8]) -> Result<(), Self::Error> {
        Err(FtdiSpiError::NotSupported("transfer_in_place"))
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_hal_async::spi::SpiDevice<u8> for FtdiSpiDevice {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        let batches = Self::batches(self.tck_init_value, self.is_lsb, operations);
        let response = FtdiMpsse::with_controller(self.mtx.clone(), move |lock| {
            Self::run_batches(lock, batches)
        })
        .await?;
        Self::copy_response(operations, &response);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asymmetric_transfers_emit_only_requested_response_bytes() {
        for (read, write, expected) in [
            (2, &[7][..], &[0x31, 0, 0, 7, 0x20, 0, 0, 0x87][..]),
            (1, &[7, 8], &[0x31, 0, 0, 7, 0x11, 0, 0, 8, 0x87]),
            (0, &[7], &[0x11, 0, 0, 7, 0x87]),
            (1, &[], &[0x20, 0, 0, 0x87]),
            (0, &[], &[0x87]),
        ] {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.transfer_bytes(false, false, read, write);
            let (bytes, response) = cmd.destruct();
            assert_eq!(bytes, expected);
            assert_eq!(response.len(), read);
        }
    }

    #[test]
    fn delay_splits_batches_and_following_reads_stay_aligned() {
        let (mut first, mut second) = ([0], [0; 2]);
        let mut ops = [
            Operation::Transfer(&mut first, &[1, 2]),
            Operation::DelayNs(500),
            Operation::Read(&mut second),
        ];
        let mut batches = FtdiSpiDevice::batches(false, false, &ops).into_iter();
        let first_batch = batches.next().unwrap();
        assert_eq!(first_batch.delay_ns, Some(500));
        assert_eq!(first_batch.cmd.destruct().1.len(), 1);
        let last_batch = batches.next().unwrap();
        assert_eq!(last_batch.delay_ns, None);
        assert_eq!(last_batch.cmd.destruct().1.len(), 2);
        assert!(batches.next().is_none());
        FtdiSpiDevice::copy_response(&mut ops, &[3, 4, 5]);
        assert_eq!(first, [3]);
        assert_eq!(second, [4, 5]);
    }
}
