//! SWD protocol support for ARM debug access over FTDI.

use std::sync::{Arc, Mutex};

use self::cmd::SwdCmdBuilder;
use crate::{
    FtdiError, Pin,
    gpio::UsedPin,
    mpsse::{FtdiMpsse, PinUsage},
};

#[derive(Debug, thiserror::Error)]
pub enum FtdiSwdError {
    #[error("Ftdi inner error")]
    FtdiInner(#[from] FtdiError),
    #[error("Swd ack wait.")]
    AckWait,
    #[error("Swd ack failed.")]
    AckFailed,
    #[error("Swd unknown ack LSB[{0:#3b}].")]
    UnknownAck(u8),
    #[error("Swd parity error.")]
    ParityError,
}

#[derive(Debug, Clone, Copy)]
pub enum SwdAddr {
    Dp(u8),
    Ap(u8),
}
impl From<SwdAddr> for u8 {
    fn from(value: SwdAddr) -> Self {
        const AP_MASK: u8 = 1 << 1;
        const ADDR_MASK: u8 = 0b11 << 3;
        match value {
            SwdAddr::Dp(addr) => (addr << 1) & ADDR_MASK,
            SwdAddr::Ap(addr) => (addr << 1) & ADDR_MASK | AP_MASK,
        }
    }
}

pub struct FtdiSwd {
    _pins: [UsedPin; 3],

    mtx: Arc<Mutex<FtdiMpsse>>,

    direction_pin: Option<UsedPin>,
}
impl FtdiSwd {
    const REPONSE_SUCCESS: u8 = 0b001;
    const REPONSE_WAIT: u8 = 0b010;
    const REPONSE_FAILED: u8 = 0b100;

    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiSwdError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::Swd)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::Swd)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUsage::Swd)?,
            ],
            mtx,
            direction_pin: None,
        };
        Ok(this)
    }
    pub fn set_direction_pin(&mut self, pin: Pin) -> Result<(), FtdiSwdError> {
        self.direction_pin = Some(UsedPin::new(self.mtx.clone(), pin, PinUsage::Swd)?);
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

    pub fn enable(&self) -> Result<(), FtdiSwdError> {
        let lock = self.mtx.lock().unwrap();
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.swd_enable();

        lock.exec(cmd)?;
        Ok(())
    }

    fn build_request(is_read: bool, addr: SwdAddr) -> u8 {
        const START_MASK: u8 = 1 << 0;
        const READ_MASK: u8 = 1 << 2;
        const PARITY_MASK: u8 = 1 << 5;
        const PARK_MASK: u8 = 1 << 7;
        let mut request = START_MASK | PARK_MASK;
        request |= if is_read { READ_MASK } else { 0 };
        request |= u8::from(addr);

        let parity = ((request >> 1) & 0x0F).count_ones() & 1 != 0;
        request |= if parity { PARITY_MASK } else { 0 };

        request
    }

    pub fn read(&self, addr: SwdAddr) -> Result<u32, FtdiSwdError> {
        let lock = self.mtx.lock().unwrap();
        let request = Self::build_request(true, addr);

        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.swd_send_request(request).trn().swd_read_response();
        let response = lock.exec(cmd)?;

        let ack = response[0] >> 5;
        if ack != Self::REPONSE_SUCCESS {
            let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin.as_deref());
            cmd.trn();
            lock.exec(cmd)?;
            return match ack {
                Self::REPONSE_WAIT => Err(FtdiSwdError::AckWait),
                Self::REPONSE_FAILED => Err(FtdiSwdError::AckFailed),
                x => Err(FtdiSwdError::UnknownAck(x)),
            };
        }

        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.swd_read_data().trn();
        let response = lock.exec(cmd)?;

        let value = u32::from_le_bytes([response[0], response[1], response[2], response[3]]);
        let parity = (response[4] >> 7) & 0x01;
        let calc_parity = value.count_ones() as u8 & 0x01;

        if parity != calc_parity {
            return Err(FtdiSwdError::ParityError);
        }
        Ok(value)
    }

    pub fn write(&self, addr: SwdAddr, value: u32) -> Result<(), FtdiSwdError> {
        let lock = self.mtx.lock().unwrap();
        let request = Self::build_request(false, addr);
        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.swd_send_request(request)
            .trn()
            .swd_read_response()
            .trn();
        let response = lock.exec(cmd)?;

        let ack = response[0] >> 5;
        if ack != Self::REPONSE_SUCCESS {
            return match ack {
                Self::REPONSE_WAIT => Err(FtdiSwdError::AckWait),
                Self::REPONSE_FAILED => Err(FtdiSwdError::AckFailed),
                x => Err(FtdiSwdError::UnknownAck(x)),
            };
        }

        let mut cmd = SwdCmdBuilder::new(&lock, self.direction_pin.as_deref());
        cmd.swd_write_data(value);
        lock.exec(cmd)?;
        Ok(())
    }
}

mod cmd {
    const SWCLK: u8 = Pin::Lower(0).mask();
    const SWDIO: u8 = Pin::Lower(1).mask();
    const TCK_INIT_VALUE: bool = false;
    const IS_LSB: bool = true;

    use crate::{Pin, mpsse::FtdiMpsse, mpsse_cmd::MpsseCmdBuilder};
    use std::sync::MutexGuard;
    pub(super) struct SwdCmdBuilder<'a> {
        cmd: MpsseCmdBuilder,
        lock: &'a MutexGuard<'a, FtdiMpsse>,
        direction_pin: Option<Pin>,
    }
    impl<'a> From<SwdCmdBuilder<'a>> for MpsseCmdBuilder {
        fn from(value: SwdCmdBuilder<'a>) -> Self {
            value.cmd
        }
    }
    impl<'a> SwdCmdBuilder<'a> {
        pub(super) fn new(lock: &'a MutexGuard<FtdiMpsse>, direction_pin: Option<&Pin>) -> Self {
            SwdCmdBuilder {
                cmd: MpsseCmdBuilder::new(),
                lock,
                direction_pin: direction_pin.copied(),
            }
        }
        fn swd_out(&mut self) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            if let Some(pin) = self.direction_pin {
                match pin {
                    Pin::Lower(_) => {
                        self.cmd.set_gpio_lower(
                            lower_value | pin.mask(),
                            lower_direction | SWCLK | SWDIO,
                        );
                    }
                    Pin::Upper(_) => {
                        self.cmd
                            .set_gpio_lower(lower_value, lower_direction | SWCLK | SWDIO);
                        self.cmd
                            .set_gpio_upper(upper_value | pin.mask(), upper_direction);
                    }
                }
            } else {
                self.cmd
                    .set_gpio_lower(lower_value, lower_direction | SWCLK | SWDIO);
            }
            self
        }
        fn swd_in(&mut self) -> &mut Self {
            let lower_value = self.lock.lower.value;
            let lower_direction = self.lock.lower.direction;
            let upper_value = self.lock.upper.value;
            let upper_direction = self.lock.upper.direction;
            if let Some(Pin::Upper(_)) = self.direction_pin {
                self.cmd.set_gpio_upper(upper_value, upper_direction);
            }
            self.cmd
                .set_gpio_lower(lower_value, lower_direction | SWCLK);
            self
        }
        pub(super) fn trn(&mut self) -> &mut Self {
            self.swd_in()
                .cmd
                .shift_bits_out(TCK_INIT_VALUE, IS_LSB, 0xff, 1);
            self
        }
        pub(super) fn swd_line_reset(&mut self) -> &mut Self {
            const ONES: &[u8] = &[0xff; 7];
            const ZEOS: u8 = 0;
            self.swd_out()
                .cmd
                .shift_bytes_out(TCK_INIT_VALUE, IS_LSB, ONES)
                .shift_bits_out(TCK_INIT_VALUE, IS_LSB, ZEOS, 2);
            self
        }
        pub(super) fn swd_enable(&mut self) -> &mut Self {
            const ONES: &[u8] = &[0xff; 7];

            const SEQUENCE: &[u8] = &0xE79E_u16.to_le_bytes();
            self.swd_out()
                .cmd
                .shift_bytes_out(TCK_INIT_VALUE, IS_LSB, ONES)
                .shift_bytes_out(TCK_INIT_VALUE, IS_LSB, SEQUENCE);
            self.swd_line_reset();
            self
        }
        pub(super) fn swd_send_request(&mut self, request: u8) -> &mut Self {
            self.swd_out()
                .cmd
                .shift_bytes_out(TCK_INIT_VALUE, IS_LSB, &[request]);
            self
        }
        pub(super) fn swd_read_response(&mut self) -> &mut Self {
            const RESPONSE_BITS: usize = 3;
            self.swd_in()
                .cmd
                .shift_bits_in(TCK_INIT_VALUE, IS_LSB, RESPONSE_BITS);
            self
        }
        pub(super) fn swd_read_data(&mut self) -> &mut Self {
            const DATA_BYTES: usize = 4;
            const PARITY_BITS: usize = 1;
            self.swd_in()
                .cmd
                .shift_bytes_in(TCK_INIT_VALUE, IS_LSB, DATA_BYTES)
                .shift_bits_in(TCK_INIT_VALUE, IS_LSB, PARITY_BITS);
            self
        }
        pub(super) fn swd_write_data(&mut self, value: u32) -> &mut Self {
            const PARITY_BITS: usize = 1;
            let bytes = value.to_le_bytes();
            let parity = (value.count_ones() & 0x01) as u8;
            self.swd_out()
                .cmd
                .shift_bytes_out(TCK_INIT_VALUE, IS_LSB, &bytes)
                .shift_bits_out(TCK_INIT_VALUE, IS_LSB, parity, PARITY_BITS);
            self
        }
    }
}
#[cfg(test)]
mod test {
    use crate::swd::{FtdiSwd, SwdAddr};

    #[test]
    fn read_id() {
        let request = FtdiSwd::build_request(true, SwdAddr::Dp(0));
        assert_eq!(request, 0xa5);
    }
}
