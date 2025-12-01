use self::cmd::I2cCmdBuilder;
use crate::{
    ChipType, FtdiError, Pin,
    gpio::UsedPin,
    mpsse::{FtdiMpsse, PinUsage},
    mpsse_cmd::MpsseCmdBuilder,
};
use eh1::i2c::{ErrorKind, NoAcknowledgeSource, Operation, SevenBitAddress};
use futures_lite::future::block_on;
use std::sync::{Arc, Mutex};

#[derive(Debug, thiserror::Error)]
pub enum FtdiI2cError {
    #[error(transparent)]
    FtdiInner(#[from] FtdiError),
    #[error("Slave not ack.")]
    NoAck(NoAcknowledgeSource),
}
/// Inter-Integrated Circuit (I2C) master controller using FTDI MPSSE
///
/// Implements I2C bus communication with support for start/stop conditions and clock stretching
pub struct FtdiI2c {
    _pins: [UsedPin; 3],
    /// Thread-safe handle to FTDI MPSSE controller
    mtx: Arc<Mutex<FtdiMpsse>>,
    /// Length of start, repeated start, and stop conditions in MPSSE commands
    /// More commands increase the duration of these conditions
    start_stop_cmds: usize,
    /// Optional direction pin for SDA line direction control (if used)
    direction_pin: Option<UsedPin>,
    enable_fast: bool,
}

impl Drop for FtdiI2c {
    fn drop(&mut self) {
        let lock = self.mtx.lock().unwrap();
        if lock.chip_type != ChipType::FT2232D {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.enable_3phase_data_clocking(false);
            lock.exec(cmd).unwrap();
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
    pub fn enbale_fast(&mut self, enable: bool) {
        self.enable_fast = enable;
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
        // lock at the start to prevent GPIO from being modified while we build
        // the MPSSE command
        let lock = self.mtx.lock().unwrap();

        // start
        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.start(self.start_stop_cmds);
        lock.exec(cmd)?;

        let mut prev_op_was_a_read = false;
        for (op_idx, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if op_idx == 0 || !prev_op_was_a_read {
                        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                        if op_idx != 0 {
                            cmd.restart(self.start_stop_cmds); // repeated start
                        }
                        cmd.i2c_addr(address, true); // (Address+Read)+Ack
                        let response = lock.exec(cmd)?;
                        if (response[0] & Self::SLAVE_ACK_MASK) == Self::SLAVE_NOT_ACK {
                            let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                            cmd.end(self.start_stop_cmds);
                            lock.exec(cmd)?;
                            return Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Address));
                        }
                    }

                    let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                    for idx in 0..buffer.len() {
                        if idx == buffer.len() - 1 {
                            cmd.i2c_read_byte(false); // NMAK: Master Not Ack
                        } else {
                            cmd.i2c_read_byte(true); // MAK: Master Ack
                        }
                    }
                    let response = lock.exec(cmd)?;
                    buffer.copy_from_slice(&response);

                    prev_op_was_a_read = true;
                }
                Operation::Write(bytes) => {
                    if op_idx == 0 || prev_op_was_a_read {
                        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                        if op_idx != 0 {
                            cmd.restart(self.start_stop_cmds); // repeated start
                        }
                        cmd.i2c_addr(address, false); // (Address+Write)+Ack
                        let response = lock.exec(cmd)?;
                        if (response[0] & Self::SLAVE_ACK_MASK) == Self::SLAVE_NOT_ACK {
                            let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                            cmd.end(self.start_stop_cmds);
                            lock.exec(cmd)?;
                            return Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Address));
                        }
                    }
                    for idx in 0..bytes.len() {
                        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                        cmd.i2c_write_byte(bytes[idx]);
                        let response = lock.exec(cmd)?;
                        if (response[0] & Self::SLAVE_ACK_MASK) == Self::SLAVE_NOT_ACK
                            && idx != bytes.len() - 1
                        {
                            let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
                            cmd.end(self.start_stop_cmds);
                            lock.exec(cmd)?;
                            return Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Data));
                        }
                    }
                    prev_op_was_a_read = false;
                }
            }
        }

        // stop
        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.end(self.start_stop_cmds);
        lock.exec(cmd)?;

        Ok(())
    }
    async fn transaction_async(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), FtdiI2cError> {
        // lock at the start to prevent GPIO from being modified while we build
        // the MPSSE command
        let lock = self.mtx.lock().unwrap();

        // start
        let mut cmd = I2cCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.start(self.start_stop_cmds);

        let mut prev_op_was_a_read = false;
        for (idx, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if idx == 0 || !prev_op_was_a_read {
                        if idx != 0 {
                            cmd.start(self.start_stop_cmds); // repeated start
                        }
                        cmd.i2c_addr(address, true);
                    }
                    for idx in 0..buffer.len() {
                        if idx == buffer.len() - 1 {
                            cmd.i2c_read_byte(false);
                        } else {
                            cmd.i2c_read_byte(true);
                        }
                    }
                    prev_op_was_a_read = true;
                }
                Operation::Write(bytes) => {
                    if idx == 0 || prev_op_was_a_read {
                        if idx != 0 {
                            cmd.start(self.start_stop_cmds); // repeated start
                        }
                        cmd.i2c_addr(address, false);
                    }
                    for &byte in *bytes {
                        cmd.i2c_write_byte(byte);
                    }
                    prev_op_was_a_read = false;
                }
            }
        }
        cmd.end(self.start_stop_cmds);

        let response = lock.exec_async(cmd).await?;

        // parse response
        prev_op_was_a_read = false;
        let mut response_idx = 0;
        for (op_idx, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buffer) => {
                    if op_idx == 0 || !prev_op_was_a_read {
                        // addr + ack_read
                        if response[response_idx] & Self::SLAVE_ACK_MASK == Self::SLAVE_NOT_ACK {
                            return Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Address));
                        }
                        response_idx += 1;
                    }
                    buffer.copy_from_slice(&response[response_idx..response_idx + buffer.len()]);
                    response_idx += buffer.len();
                    prev_op_was_a_read = true;
                }
                Operation::Write(bytes) => {
                    if op_idx == 0 || prev_op_was_a_read {
                        if response[response_idx] & Self::SLAVE_ACK_MASK == Self::SLAVE_NOT_ACK {
                            return Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Address));
                        }
                        response_idx += 1;
                    }
                    for idx in 0..bytes.len() {
                        if idx != bytes.len() - 1
                            && response[response_idx] & Self::SLAVE_ACK_MASK == Self::SLAVE_NOT_ACK
                        {
                            return Err(FtdiI2cError::NoAck(NoAcknowledgeSource::Data));
                        }

                        response_idx += 1;
                    }
                    prev_op_was_a_read = false;
                }
            }
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

/// I2C trait implementation for FTDI MPSSE
impl eh1::i2c::I2c for FtdiI2c {
    /// Executes a sequence of I2C operations (read/write) on the specified device
    ///
    /// # Arguments
    /// * `address` - 7-bit I2C device address
    /// * `operations` - Slice of read/write operations to perform
    ///
    /// # Returns
    /// Result indicating success or failure of the transaction
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if self.enable_fast {
            block_on(self.transaction_async(address, operations))
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
        self.transaction_async(address, operations).await
    }
}

mod cmd {
    const SCL: u8 = Pin::Lower(0).mask(); // SCK bitmask
    const SDA: u8 = Pin::Lower(1).mask(); // DIO bitmask
    const TCK_INIT_VALUE: bool = false;
    const IS_LSB: bool = false;
    const DATA_BITS: usize = 8;
    const ACK_BITS: usize = 1;

    use crate::{Pin, mpsse::FtdiMpsse, mpsse_cmd::MpsseCmdBuilder};
    use std::sync::MutexGuard;
    pub(super) struct I2cCmdBuilder<'a> {
        cmd: MpsseCmdBuilder,
        lock: &'a MutexGuard<'a, FtdiMpsse>,
        direction_pin: Option<Pin>,
    }
    impl<'a> From<I2cCmdBuilder<'a>> for MpsseCmdBuilder {
        fn from(value: I2cCmdBuilder<'a>) -> Self {
            value.cmd
        }
    }
    impl<'a> I2cCmdBuilder<'a> {
        pub(super) fn new(lock: &'a MutexGuard<FtdiMpsse>, direction_pin: Option<&Pin>) -> Self {
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
        pub(super) fn i2c_read_byte(&mut self, m_ack: bool) -> &mut Self {
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
