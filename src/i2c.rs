//! I2C master implementation built on FTDI MPSSE.

use self::cmd::I2cCmdBuilder;
use crate::{
    ChipType, FtdiError, Pin,
    gpio::UsedPin,
    mpsse::{FtdiMpsse, PinUsage},
    mpsse_cmd::MpsseCmdBuilder,
};
use eh1::i2c::{ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};
use std::sync::{Arc, Mutex};

#[derive(Debug, thiserror::Error)]
pub enum FtdiI2cError {
    #[error(transparent)]
    FtdiInner(#[from] FtdiError),
    #[error("Slave not ack.")]
    NoAck(NoAcknowledgeSource),
}

pub struct FtdiI2c {
    _pins: [UsedPin; 3],

    mtx: Arc<Mutex<FtdiMpsse>>,

    start_stop_cmds: usize,

    direction_pin: Option<UsedPin>,
    enable_fast: bool,
}

// Adjacent operations with the same direction are one I²C address phase.
enum Phase {
    Read(usize),
    Write(Vec<u8>),
}

fn phases(operations: &[Operation<'_>]) -> Vec<Phase> {
    let mut phases = Vec::new();
    for op in operations {
        match (op, phases.last_mut()) {
            (Operation::Read(buf), Some(Phase::Read(len))) => *len += buf.len(),
            (Operation::Write(buf), Some(Phase::Write(bytes))) => bytes.extend_from_slice(buf),
            (Operation::Read(buf), _) => phases.push(Phase::Read(buf.len())),
            (Operation::Write(buf), _) => phases.push(Phase::Write(buf.to_vec())),
        }
    }
    phases
}
impl Drop for FtdiI2c {
    fn drop(&mut self) {
        let lock = self.mtx.lock().unwrap();
        if lock.chip_type != ChipType::FT2232D {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.enable_3phase_data_clocking(false);
            if let Err(e) = lock.exec(cmd) {
                log::warn!("Failed to disable 3-phase clocking while dropping I2C controller: {e}");
            }
        }
    }
}

impl FtdiI2c {
    const SLAVE_ACK_MASK: u8 = 1 << 0;
    const SLAVE_NOT_ACK: u8 = Self::SLAVE_ACK_MASK;
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiI2cError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::I2c)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::I2c)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUsage::I2c)?,
            ],
            mtx: mtx.clone(),
            start_stop_cmds: 3,
            direction_pin: None,
            enable_fast: false,
        };
        {
            let lock = mtx.lock().unwrap();
            if lock.chip_type != ChipType::FT2232D {
                let mut cmd = MpsseCmdBuilder::new();
                cmd.enable_3phase_data_clocking(true);
                lock.exec(cmd)?;
            }
        }
        log::info!("IIC default 100Khz");
        this.set_frequency(100_000)?;
        Ok(this)
    }

    pub fn set_direction_pin(&mut self, pin: Pin) -> Result<(), FtdiI2cError> {
        self.direction_pin = Some(UsedPin::new(self.mtx.clone(), pin, PinUsage::I2c)?);
        let mut lock = self.mtx.lock().unwrap();
        match self.direction_pin.as_deref().unwrap() {
            Pin::Lower(_) => {
                lock.lower.direction |= pin.mask();
            }
            Pin::Upper(_) => {
                lock.upper.direction |= pin.mask();
            }
        }
        Ok(())
    }
    pub fn enable_fast(&mut self, enable: bool) {
        self.enable_fast = enable;
    }

    #[deprecated(note = "Use enable_fast instead")]
    pub fn enbale_fast(&mut self, enable: bool) {
        self.enable_fast(enable);
    }

    pub fn set_stop_start_len(&mut self, start_stop_cmds: usize) {
        self.start_stop_cmds = start_stop_cmds
    }

    pub fn set_frequency(&self, frequency_hz: usize) -> Result<(), FtdiI2cError> {
        let lock = self.mtx.lock().unwrap();
        if lock.chip_type == ChipType::FT2232D {
            lock.set_frequency(frequency_hz)?;
        } else {
            lock.set_frequency(frequency_hz * 3 / 2)?;
        }
        Ok(())
    }

    pub fn scan(&mut self) -> Vec<u8> {
        let mut addr_set = Vec::new();
        for addr in 0..128 {
            let write_response = self.transaction(addr, &mut [Operation::Write(&[])]);
            let read_response = self.transaction(addr, &mut [Operation::Read(&mut [])]);
            if write_response.is_ok() || read_response.is_ok() {
                addr_set.push(addr);
            }
        }
        addr_set
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), FtdiI2cError> {
        let lock = self.mtx.lock().unwrap();

        let command = || I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
        let send_ack = |cmd: I2cCmdBuilder, source| {
            let response = lock.exec(cmd)?;
            if let Err(error) = Self::check_ack(response[0], source) {
                let mut stop = command();
                stop.end(self.start_stop_cmds);
                lock.exec(stop)?;
                return Err(error);
            }
            Ok(())
        };

        let mut start = command();
        start.start(self.start_stop_cmds);
        lock.exec(start)?;

        for (index, group) in operations
            .chunk_by_mut(|a, b| matches!(a, Operation::Read(_)) == matches!(b, Operation::Read(_)))
            .enumerate()
        {
            let mut cmd = command();
            if index > 0 {
                cmd.restart(self.start_stop_cmds);
            }
            cmd.i2c_addr(address, matches!(group[0], Operation::Read(_)));
            send_ack(cmd, NoAcknowledgeSource::Address)?;

            let last_read = group
                .iter()
                .rposition(|op| matches!(op, Operation::Read(buf) if !buf.is_empty()));
            for (idx, op) in group.iter_mut().enumerate() {
                match op {
                    Operation::Read(buf) if !buf.is_empty() => {
                        let mut cmd = command();
                        cmd.read_bytes(buf.len(), Some(idx) != last_read);
                        buf.copy_from_slice(&lock.exec(cmd)?);
                    }
                    Operation::Read(_) => {}
                    Operation::Write(bytes) => {
                        for &byte in bytes.iter() {
                            let mut cmd = command();
                            cmd.i2c_write_byte(byte);
                            send_ack(cmd, NoAcknowledgeSource::Data)?;
                        }
                    }
                }
            }
        }
        let mut stop = command();
        stop.end(self.start_stop_cmds);
        lock.exec(stop)?;
        Ok(())
    }
    fn fast_commands(
        lock: &FtdiMpsse,
        address: u8,
        phases: &[Phase],
        direction: Option<&Pin>,
        count: usize,
    ) -> MpsseCmdBuilder {
        let mut cmd = I2cCmdBuilder::new(lock, direction);
        cmd.start(count);
        for (idx, phase) in phases.iter().enumerate() {
            if idx > 0 {
                cmd.restart(count);
            }
            cmd.i2c_addr(address, matches!(phase, Phase::Read(_)));
            match phase {
                Phase::Read(len) => cmd.read_bytes(*len, false),
                Phase::Write(bytes) => {
                    for &byte in bytes {
                        cmd.i2c_write_byte(byte);
                    }
                }
            }
        }
        cmd.end(count);
        cmd.into()
    }

    fn parse_response(
        operations: &mut [Operation<'_>],
        response: &[u8],
    ) -> Result<(), FtdiI2cError> {
        let mut remaining = response;
        let mut previous_read = None;
        for op in operations {
            let read = matches!(op, Operation::Read(_));
            if previous_read != Some(read) {
                Self::check_ack(remaining[0], NoAcknowledgeSource::Address)?;
                remaining = &remaining[1..];
            }
            previous_read = Some(read);
            match op {
                Operation::Read(buf) => {
                    let (data, rest) = remaining.split_at(buf.len());
                    buf.copy_from_slice(data);
                    remaining = rest;
                }
                Operation::Write(bytes) => {
                    let (acks, rest) = remaining.split_at(bytes.len());
                    for &ack in acks {
                        Self::check_ack(ack, NoAcknowledgeSource::Data)?;
                    }
                    remaining = rest;
                }
            }
        }
        Ok(())
    }

    fn check_ack(ack: u8, source: NoAcknowledgeSource) -> Result<(), FtdiI2cError> {
        if ack & Self::SLAVE_ACK_MASK == Self::SLAVE_NOT_ACK {
            return Err(FtdiI2cError::NoAck(source));
        }
        Ok(())
    }
}

impl eh1::i2c::Error for FtdiI2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            FtdiI2cError::NoAck(x) => ErrorKind::NoAcknowledge(*x),
            _ => ErrorKind::Other,
        }
    }
}

impl eh1::i2c::ErrorType for FtdiI2c {
    type Error = FtdiI2cError;
}

impl eh1::i2c::I2c for FtdiI2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if self.enable_fast {
            let lock = self.mtx.lock().unwrap();
            let cmd = Self::fast_commands(
                &lock,
                address,
                &phases(operations),
                self.direction_pin.as_deref(),
                self.start_stop_cmds,
            );
            Self::parse_response(operations, &lock.exec(cmd)?)
        } else {
            self.transaction(address, operations)
        }
    }
}

impl embedded_hal_async::i2c::I2c for FtdiI2c {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let phases = phases(operations);
        let direction = self.direction_pin.as_deref().copied();
        let count = self.start_stop_cmds;
        let response = FtdiMpsse::with_controller(self.mtx.clone(), move |lock| {
            lock.exec(Self::fast_commands(
                lock,
                address,
                &phases,
                direction.as_ref(),
                count,
            ))
        })
        .await?;
        Self::parse_response(operations, &response)
    }
}

mod cmd {
    const SCL: u8 = Pin::Lower(0).mask();
    const SDA: u8 = Pin::Lower(1).mask();
    const TCK_INIT_VALUE: bool = false;
    const IS_LSB: bool = false;
    const DATA_BITS: usize = 8;
    const ACK_BITS: usize = 1;

    use crate::{Pin, mpsse::FtdiMpsse, mpsse_cmd::MpsseCmdBuilder};
    pub(super) struct I2cCmdBuilder<'a> {
        cmd: MpsseCmdBuilder,
        lock: &'a FtdiMpsse,
        direction_pin: Option<Pin>,
    }
    impl<'a> From<I2cCmdBuilder<'a>> for MpsseCmdBuilder {
        fn from(value: I2cCmdBuilder<'a>) -> Self {
            value.cmd
        }
    }
    impl<'a> I2cCmdBuilder<'a> {
        pub(super) fn new(lock: &'a FtdiMpsse, direction_pin: Option<&Pin>) -> Self {
            I2cCmdBuilder {
                cmd: MpsseCmdBuilder::new(),
                lock,
                direction_pin: direction_pin.copied(),
            }
        }
        fn i2c_out(&mut self, scl: bool, sda: bool) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            let scl = if scl { SCL } else { 0 };
            let sda = if sda { SDA } else { 0 };
            if let Some(pin) = self.direction_pin {
                match pin {
                    Pin::Lower(_) => {
                        self.cmd.set_gpio_lower(
                            lower_value | pin.mask() | scl | sda,
                            lower_direction | SCL | SDA,
                        );
                    }
                    Pin::Upper(_) => {
                        self.cmd
                            .set_gpio_lower(lower_value | scl | sda, lower_direction | SCL | SDA);
                        self.cmd
                            .set_gpio_upper(upper_value | pin.mask(), upper_direction);
                    }
                }
            } else {
                self.cmd
                    .set_gpio_lower(lower_value | scl | sda, lower_direction | SCL | SDA);
            }
            self
        }
        fn i2c_in(&mut self) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            if let Some(Pin::Upper(_)) = self.direction_pin {
                self.cmd.set_gpio_upper(upper_value, upper_direction);
            }
            self.cmd.set_gpio_lower(lower_value, lower_direction | SCL);
            self
        }
        pub(super) fn start(&mut self, count: usize) -> &mut Self {
            for _ in 0..count {
                self.i2c_out(true, true);
            }
            for _ in 0..count {
                self.i2c_out(true, false);
            }
            for _ in 0..count {
                self.i2c_out(false, false);
            }
            self
        }
        pub(super) fn restart(&mut self, count: usize) -> &mut Self {
            for _ in 0..count {
                self.i2c_out(false, true);
            }
            self.start(count)
        }
        pub(super) fn end(&mut self, count: usize) -> &mut Self {
            for _ in 0..count {
                self.i2c_out(false, false);
            }
            for _ in 0..count {
                self.i2c_out(true, false);
            }
            for _ in 0..count {
                self.i2c_out(true, true);
            }
            self
        }
        pub(super) fn i2c_addr(&mut self, addr: u8, is_read: bool) -> &mut Self {
            let addr = if is_read { (addr << 1) | 1 } else { addr << 1 };
            self.cmd
                .shift_bits_out(TCK_INIT_VALUE, IS_LSB, addr, DATA_BITS);
            self.i2c_in()
                .cmd
                .shift_bits_in(TCK_INIT_VALUE, IS_LSB, ACK_BITS);
            self
        }
        pub(super) fn read_bytes(&mut self, len: usize, continues: bool) {
            for idx in 0..len {
                self.i2c_read_byte(continues || idx + 1 < len);
            }
        }
        fn i2c_read_byte(&mut self, m_ack: bool) -> &mut Self {
            let m_ack = if m_ack { 0 } else { 0xff };
            self.i2c_in()
                .cmd
                .shift_bits_in(TCK_INIT_VALUE, IS_LSB, DATA_BITS);
            self.i2c_out(false, false)
                .cmd
                .shift_bits_out(TCK_INIT_VALUE, IS_LSB, m_ack, ACK_BITS);
            self
        }
        pub(super) fn i2c_write_byte(&mut self, value: u8) -> &mut Self {
            self.i2c_out(false, false)
                .cmd
                .shift_bits_out(TCK_INIT_VALUE, IS_LSB, value, DATA_BITS);
            self.i2c_in()
                .cmd
                .shift_bits_in(TCK_INIT_VALUE, IS_LSB, ACK_BITS);
            self
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn last_write_byte_must_be_acknowledged() {
        let mut ops = [Operation::Write(&[0x11]), Operation::Write(&[0x22])];
        for response in [[0, 1, 0], [0, 0, 1]] {
            assert!(matches!(
                FtdiI2c::parse_response(&mut ops, &response),
                Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Data))
            ));
        }
        assert!(matches!(
            FtdiI2c::parse_response(&mut ops, &[1, 0, 0]),
            Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Address))
        ));
    }

    #[test]
    fn adjacent_reads_share_one_address_phase() {
        let (mut a, mut b) = ([0; 1], [0; 2]);
        let mut ops = [
            Operation::Write(&[7]),
            Operation::Read(&mut a),
            Operation::Read(&mut []),
            Operation::Read(&mut b),
            Operation::Write(&[]),
        ];
        assert!(matches!(
            &phases(&ops)[..],
            [Phase::Write(_), Phase::Read(3), Phase::Write(_)]
        ));
        FtdiI2c::parse_response(&mut ops, &[0, 0, 0, 1, 2, 3, 0]).unwrap();
        assert_eq!(a, [1]);
        assert_eq!(b, [2, 3]);
    }
}
