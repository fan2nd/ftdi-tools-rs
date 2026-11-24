//! GPIO wrappers that implement embedded-hal digital traits.

use crate::{
    FtdiError, Pin,
    mpsse::{FtdiMpsse, PinUsage},
    mpsse_cmd::MpsseCmdBuilder,
};
use std::{
    ops::Deref,
    sync::{Arc, Mutex},
};

pub(crate) struct UsedPin {
    mtx: Arc<Mutex<FtdiMpsse>>,

    pin: Pin,
}
impl Drop for UsedPin {
    fn drop(&mut self) {
        let mut lock = self.mtx.lock().unwrap();
        lock.free_pin(self.pin);
    }
}
impl Deref for UsedPin {
    type Target = Pin;
    fn deref(&self) -> &Self::Target {
        &self.pin
    }
}
impl UsedPin {
    pub(crate) fn new(
        mtx: Arc<Mutex<FtdiMpsse>>,
        pin: Pin,
        usage: PinUsage,
    ) -> Result<Self, FtdiError> {
        {
            let mut lock = mtx.lock().unwrap();
            lock.alloc_pin(pin, usage)?;
        }
        Ok(Self { mtx, pin })
    }
}

pub struct FtdiOutputPin {
    mtx: Arc<Mutex<FtdiMpsse>>,

    pin: UsedPin,
}

impl FtdiOutputPin {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, pin: Pin) -> Result<Self, FtdiError> {
        let this = Self {
            mtx: mtx.clone(),
            pin: UsedPin::new(mtx.clone(), pin, PinUsage::Output)?,
        };
        {
            let mut lock = mtx.lock().unwrap();
            let mut cmd = MpsseCmdBuilder::new();
            match pin {
                Pin::Lower(_) => {
                    lock.lower.direction |= pin.mask();
                    cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
                }
                Pin::Upper(_) => {
                    lock.upper.direction |= pin.mask();
                    cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
                }
            }
            lock.exec(cmd)?;
        }
        Ok(this)
    }
}

impl eh1::digital::Error for FtdiError {
    fn kind(&self) -> eh1::digital::ErrorKind {
        eh1::digital::ErrorKind::Other
    }
}

impl eh1::digital::ErrorType for FtdiOutputPin {
    type Error = FtdiError;
}

impl eh1::digital::OutputPin for FtdiOutputPin {
    fn set_low(&mut self) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        match *self.pin {
            Pin::Lower(_) => {
                lock.lower.value &= !self.pin.mask();
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            Pin::Upper(_) => {
                lock.upper.value &= !self.pin.mask();
                cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
            }
        }
        lock.exec(cmd)?;

        Ok(())
    }

    fn set_high(&mut self) -> Result<(), FtdiError> {
        let mut lock = self.mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        match *self.pin {
            Pin::Lower(_) => {
                lock.lower.value |= self.pin.mask();
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            Pin::Upper(_) => {
                lock.upper.value |= self.pin.mask();
                cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
            }
        }
        lock.exec(cmd)?;

        Ok(())
    }
}

pub struct FtdiInputPin {
    mtx: Arc<Mutex<FtdiMpsse>>,

    pin: UsedPin,
}

impl FtdiInputPin {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>, pin: Pin) -> Result<Self, FtdiError> {
        let this = Self {
            mtx: mtx.clone(),
            pin: UsedPin::new(mtx.clone(), pin, PinUsage::Input)?,
        };
        let mut lock = mtx.lock().unwrap();
        let mut cmd = MpsseCmdBuilder::new();
        match pin {
            Pin::Lower(_) => {
                lock.lower.direction &= !pin.mask();
                cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            }
            Pin::Upper(_) => {
                lock.upper.direction &= !pin.mask();
                cmd.set_gpio_upper(lock.upper.value, lock.upper.direction);
            }
        }
        lock.exec(cmd)?;
        Ok(this)
    }

    pub(crate) fn get(&self) -> Result<bool, FtdiError> {
        let lock = self.mtx.lock().unwrap();

        let mut cmd = MpsseCmdBuilder::new();
        match *self.pin {
            Pin::Lower(_) => cmd.gpio_lower(),
            Pin::Upper(_) => cmd.gpio_upper(),
        };
        let response = lock.exec(cmd)?;

        Ok(response[0] & self.pin.mask() != 0)
    }
}

impl eh1::digital::ErrorType for FtdiInputPin {
    type Error = FtdiError;
}

impl eh1::digital::InputPin for FtdiInputPin {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.get()
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.get().map(|res| !res)
    }
}
