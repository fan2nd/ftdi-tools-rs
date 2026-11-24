//! Hardware JTAG driver based on FTDI MPSSE commands.

use crate::{
    ChipType, FtdiError, Pin,
    gpio::{FtdiOutputPin, UsedPin},
    mpsse::{FtdiMpsse, PinUsage},
    mpsse_cmd::MpsseCmdBuilder,
};
use eh1::digital::OutputPin;
use std::sync::{Arc, Mutex};

const TCK_MASK: u8 = Pin::Lower(0).mask();
const TDI_MASK: u8 = Pin::Lower(1).mask();
#[allow(unused)]
const TDO_MASK: u8 = Pin::Lower(2).mask();
const TMS_MASK: u8 = Pin::Lower(3).mask();

const TCK_INIT_VALUE: bool = false;
const IS_LSB: bool = true;

pub struct FtdiJtag {
    _pins: [UsedPin; 4],

    mtx: Arc<Mutex<FtdiMpsse>>,

    adaptive_clocking_pin: Option<UsedPin>,

    direction: Option<[FtdiOutputPin; 4]>,
}
impl Drop for FtdiJtag {
    fn drop(&mut self) {
        if let Err(e) = self.adaptive_clock(false) {
            log::warn!("Failed to disable adaptive clocking while dropping JTAG controller: {e}");
        }
    }
}
impl FtdiJtag {
    pub fn new(mtx: Arc<Mutex<FtdiMpsse>>) -> Result<Self, FtdiError> {
        let this = Self {
            _pins: [
                UsedPin::new(mtx.clone(), Pin::Lower(0), PinUsage::Jtag)?,
                UsedPin::new(mtx.clone(), Pin::Lower(1), PinUsage::Jtag)?,
                UsedPin::new(mtx.clone(), Pin::Lower(2), PinUsage::Jtag)?,
                UsedPin::new(mtx.clone(), Pin::Lower(3), PinUsage::Jtag)?,
            ],
            mtx: mtx.clone(),
            adaptive_clocking_pin: None,
            direction: None,
        };
        {
            let mut lock = mtx.lock().unwrap();

            lock.lower.direction |= TCK_MASK | TDI_MASK | TMS_MASK;

            let mut cmd = MpsseCmdBuilder::new();
            cmd.set_gpio_lower(lock.lower.value, lock.lower.direction);
            lock.exec(cmd)?;
        }
        Ok(this)
    }

    pub fn adaptive_clock(&mut self, state: bool) -> Result<(), FtdiError> {
        if self.adaptive_clocking_pin.is_some() == state {
            return Ok(());
        }
        {
            let lock = self.mtx.lock().unwrap();
            if lock.chip_type == ChipType::FT2232D {
                return Ok(());
            }
            let mut cmd = MpsseCmdBuilder::new();
            cmd.enable_adaptive_clocking(state);
            lock.exec(cmd)?;
        }
        if state {
            log::info!("Use {:?} as RTCK.", Pin::Lower(7));
            self.adaptive_clocking_pin = Some(UsedPin::new(
                self.mtx.clone(),
                Pin::Lower(7),
                PinUsage::Jtag,
            )?);
        } else {
            log::info!("Free {:?}.", Pin::Lower(7));
            self.adaptive_clocking_pin = None;
        }
        Ok(())
    }

    pub fn with_direction(
        &mut self,
        tck: Pin,
        tdi: Pin,
        tdo: Pin,
        tms: Pin,
    ) -> Result<(), FtdiError> {
        let mut tck = FtdiOutputPin::new(self.mtx.clone(), tck)?;
        let mut tdi = FtdiOutputPin::new(self.mtx.clone(), tdi)?;
        let mut tdo = FtdiOutputPin::new(self.mtx.clone(), tdo)?;
        let mut tms = FtdiOutputPin::new(self.mtx.clone(), tms)?;
        tck.set_high()?;
        tdi.set_high()?;
        tdo.set_low()?;
        tms.set_high()?;
        self.direction = Some([tck, tdi, tdo, tms]);
        Ok(())
    }
    pub fn goto_idle(&mut self) -> Result<(), FtdiError> {
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle();
        let lock = self.mtx.lock().unwrap();
        lock.exec(cmd)?;
        Ok(())
    }
    pub fn scan_with(&mut self, tdi: bool) -> Result<Vec<u32>, FtdiError> {
        const ID_LEN: usize = 32;
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle().jtag_idle2dr();
        let lock = self.mtx.lock().unwrap();
        lock.exec(cmd)?;
        let tdi = if tdi { vec![0xff; 4] } else { vec![0; 4] };

        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_zeros = 0;

        'outer: loop {
            let mut cmd = MpsseCmdBuilder::new();
            cmd.shift_bytes(TCK_INIT_VALUE, IS_LSB, &tdi);
            let response = lock.exec(cmd)?;
            let tdos: Vec<_> = response
                .iter()
                .flat_map(|&byte| (0..8).map(move |i| (byte >> i) & 1 == 1))
                .collect();
            for tdo_val in tdos {
                if bit_count == 0 && !tdo_val {
                    idcodes.push(0);
                    consecutive_zeros += 1;
                } else {
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_zeros = 0;
                }

                if consecutive_zeros == ID_LEN {
                    break 'outer;
                }

                if bit_count == ID_LEN {
                    if current_id == u32::MAX {
                        break 'outer;
                    }
                    idcodes.push(current_id);
                    bit_count = 0;
                }
            }
        }

        drop(lock);
        self.goto_idle()?;
        Ok(idcodes)
    }
    pub fn write(&self, ir: &[u8], irlen: usize, dr: &[u8], drlen: usize) -> Result<(), FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();

        cmd.jtag_any2idle();
        cmd.jtag_idle2ir()
            .jtag_shift_write(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift_write(dr, drlen)
            .jtag_dr_exit2idle()
            .jtag_idle_cycle();
        let lock = self.mtx.lock().unwrap();
        lock.exec(cmd)?;
        Ok(())
    }
    pub fn read(&self, ir: &[u8], irlen: usize, drlen: usize) -> Result<Vec<u8>, FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle();
        cmd.jtag_idle2ir()
            .jtag_shift_write(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift_read(drlen)
            .jtag_dr_exit2idle()
            .jtag_idle_cycle();
        let lock = self.mtx.lock().unwrap();
        let mut response = lock.exec(cmd)?;
        let len = JtagCmdBuilder::jtag_parse_single_shift(&mut response, drlen);

        if response.len() > len {
            response.pop();
        }
        Ok(response)
    }
    pub fn write_read(
        &self,
        ir: &[u8],
        irlen: usize,
        dr: &[u8],
        drlen: usize,
    ) -> Result<Vec<u8>, FtdiError> {
        log::warn!("Not test");
        let mut cmd = JtagCmdBuilder::new();
        cmd.jtag_any2idle();
        cmd.jtag_idle2ir()
            .jtag_shift_write(ir, irlen)
            .jtag_ir_exit2dr()
            .jtag_shift(dr, drlen)
            .jtag_dr_exit2idle()
            .jtag_idle_cycle();
        let lock = self.mtx.lock().unwrap();
        let mut response = lock.exec(cmd)?;
        let len = JtagCmdBuilder::jtag_parse_single_shift(&mut response, drlen);

        if response.len() > len {
            response.pop();
        }
        Ok(response)
    }
}

struct JtagCmdBuilder(MpsseCmdBuilder);
impl From<JtagCmdBuilder> for MpsseCmdBuilder {
    fn from(value: JtagCmdBuilder) -> Self {
        value.0
    }
}
impl JtagCmdBuilder {
    fn new() -> Self {
        JtagCmdBuilder(MpsseCmdBuilder::new())
    }
    fn jtag_any2idle(&mut self) -> &mut Self {
        self.0.clock_tms_out(true, 0b0001_1111, 6);
        self
    }
    fn jtag_idle_cycle(&mut self) -> &mut Self {
        self.0.clock_tms_out(true, 0, 7);
        self
    }
    fn jtag_idle2ir(&mut self) -> &mut Self {
        self.0.clock_tms_out(true, 0b0000_0011, 4);
        self
    }
    fn jtag_ir_exit2dr(&mut self) -> &mut Self {
        self.0.clock_tms_out(true, 0b0000_0011, 4);
        self
    }
    fn jtag_idle2dr(&mut self) -> &mut Self {
        self.0.clock_tms_out(true, 0b0000_0001, 3);
        self
    }
    fn jtag_dr_exit2idle(&mut self) -> &mut Self {
        self.0.clock_tms_out(true, 0b0000_0001, 2);
        self
    }
    fn jtag_shift(&mut self, data: &[u8], bits_count: usize) -> &mut Self {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        let last_bit = (data[bytes_count] >> remain_bits) & 1 != 0;
        self.0
            .shift_bytes(TCK_INIT_VALUE, IS_LSB, &data[0..bytes_count])
            .shift_bits(TCK_INIT_VALUE, IS_LSB, data[bytes_count], remain_bits)
            .clock_tms(last_bit, 0b0000_0001, 1);
        self
    }
    fn jtag_shift_write(&mut self, data: &[u8], bits_count: usize) -> &mut Self {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        let last_bit = (data[bytes_count] >> remain_bits) & 1 != 0;
        self.0
            .shift_bytes_out(TCK_INIT_VALUE, IS_LSB, &data[0..bytes_count])
            .shift_bits_out(TCK_INIT_VALUE, IS_LSB, data[bytes_count], remain_bits)
            .clock_tms_out(last_bit, 0b0000_0001, 1);
        self
    }
    fn jtag_shift_read(&mut self, bits_count: usize) -> &mut Self {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        let last_bit = Default::default();
        self.0
            .shift_bytes_in(TCK_INIT_VALUE, IS_LSB, bytes_count)
            .shift_bits_in(TCK_INIT_VALUE, IS_LSB, remain_bits)
            .clock_tms(last_bit, 0b0000_0001, 1);
        self
    }
    fn jtag_parse_single_shift(response: &mut [u8], bits_count: usize) -> usize {
        assert!(bits_count != 0);
        let bytes_count = (bits_count - 1) >> 3;
        let remain_bits = (bits_count - 1) & 0b111;
        if remain_bits == 0 {
            response[bytes_count] >>= 7
        } else {
            response[bytes_count] >>= 8 - remain_bits;
            response[bytes_count] |= (response[bytes_count + 1] & 0b1000_0000) >> (7 - remain_bits);
        }
        bytes_count + 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn padding_bits_do_not_change_last_tdi_bit() {
        for (byte, expected) in [(0xff, 0x81), (0xf7, 0x01)] {
            let mut write = JtagCmdBuilder::new();
            write.jtag_shift_write(&[byte], 4);
            let mut transfer = JtagCmdBuilder::new();
            transfer.jtag_shift(&[byte], 4);
            for cmd in [write, transfer] {
                let (bytes, _) = cmd.0.destruct();
                assert_eq!(bytes[bytes.len() - 2], expected);
            }
        }
    }
}
