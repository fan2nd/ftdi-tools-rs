//! JTAG scan-chain detection helpers.

use crate::{FtdiError, mpsse::FtdiMpsse, mpsse_cmd::MpsseCmdBuilder};

const ID_LEN: usize = 32;
pub struct JtagDetectTdo {
    mpsse: FtdiMpsse,
    tck_mask: u8,
    tms_mask: u8,
}
impl From<JtagDetectTdo> for FtdiMpsse {
    fn from(value: JtagDetectTdo) -> Self {
        value.mpsse
    }
}
impl JtagDetectTdo {
    pub fn new(mpsse: impl Into<FtdiMpsse>) -> Self {
        Self {
            mpsse: mpsse.into(),
            tck_mask: 0,
            tms_mask: 0,
        }
    }
    pub fn set_pins(&mut self, tck: usize, tms: usize) {
        self.tck_mask = 1 << tck;
        self.tms_mask = 1 << tms;
    }
    fn shift_dr(&self, len: usize) -> Result<Vec<u8>, FtdiError> {
        let direction = self.tck_mask | self.tms_mask;
        let mut cmd = MpsseCmdBuilder::new();
        for _ in 0..len {
            cmd.set_gpio_lower(0, direction)
                .set_gpio_lower(self.tck_mask, direction)
                .gpio_lower();
        }
        let response = self.mpsse.exec(cmd)?;
        Ok(response)
    }

    pub fn scan(&self) -> Result<Vec<usize>, FtdiError> {
        let mask = self.tck_mask | self.tms_mask;
        if mask.count_ones() != 2 {
            return Err(FtdiError::Other("tck cannot be same to tms."));
        }
        let mut tdo_pins = Vec::new();
        reset2dr(&self.mpsse, self.tck_mask, self.tms_mask)?;
        let read = self.shift_dr(ID_LEN * 2)?;

        for i in 0..8 {
            if 1 << i == self.tck_mask || 1 << i == self.tms_mask {
                continue;
            }
            let mut current_id = 0;
            let mut bit_count = 0;
            let mut consecutive_bypass = 0;
            let tdos: Vec<_> = read.iter().map(|&x| (x >> i) & 1 == 1).collect();

            for tdo_val in tdos {
                if bit_count == 0 && !tdo_val {
                    consecutive_bypass += 1;
                } else {
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_bypass = 0;
                }

                if consecutive_bypass == ID_LEN {
                    break;
                }

                if bit_count == ID_LEN {
                    if current_id != u32::MAX {
                        tdo_pins.push(i);
                    }
                    break;
                }
            }
        }
        Ok(tdo_pins)
    }
}

pub struct JtagDetectTdi {
    mpsse: FtdiMpsse,
    tck_mask: u8,
    tdi_mask: u8,
    tdo_mask: u8,
    tms_mask: u8,
}
impl From<JtagDetectTdi> for FtdiMpsse {
    fn from(value: JtagDetectTdi) -> Self {
        value.mpsse
    }
}
impl JtagDetectTdi {
    pub fn new(mpsse: impl Into<FtdiMpsse>) -> Self {
        Self {
            mpsse: mpsse.into(),
            tck_mask: 0,
            tdi_mask: 0,
            tdo_mask: 0,
            tms_mask: 0,
        }
    }
    pub fn set_pins(&mut self, tck: usize, tdi: usize, tdo: usize, tms: usize) {
        self.tck_mask = 1 << tck;
        self.tdi_mask = 1 << tdi;
        self.tdo_mask = 1 << tdo;
        self.tms_mask = 1 << tms;
    }
    fn shift_dr(&self, tdi_value: bool, len: usize) -> Result<Vec<bool>, FtdiError> {
        let direction = !(self.tdo_mask);
        let tdi_mask = if tdi_value { self.tdi_mask } else { 0 };
        let mut cmd = MpsseCmdBuilder::new();
        for _ in 0..len {
            cmd.set_gpio_lower(tdi_mask, direction)
                .set_gpio_lower(tdi_mask | self.tck_mask, direction)
                .gpio_lower();
        }
        let response = self.mpsse.exec(cmd)?;
        Ok(response
            .into_iter()
            .map(|x| x & (self.tdo_mask) != 0)
            .collect())
    }

    pub fn scan_with(&self, tdi_val: bool) -> Result<Vec<u32>, FtdiError> {
        let mask = self.tck_mask | self.tms_mask | self.tdi_mask | self.tdo_mask;
        if mask.count_ones() != 4 {
            return Err(FtdiError::Other(
                "any one of tck/tms/tdi/tdo cannot be same to others.",
            ));
        }
        const ID_LEN: usize = 32;

        let mut idcodes = Vec::new();
        let mut current_id = 0u32;
        let mut bit_count = 0;
        let mut consecutive_bypass = 0;

        reset2dr(&self.mpsse, self.tck_mask, self.tms_mask)?;

        'outer: loop {
            let tdos: Vec<_> = self.shift_dr(tdi_val, ID_LEN * 2)?;
            for tdo_val in tdos {
                if bit_count == 0 && !tdo_val {
                    idcodes.push(0);
                    consecutive_bypass += 1;
                } else {
                    current_id = (current_id >> 1) | if tdo_val { 0x8000_0000 } else { 0 };
                    bit_count += 1;
                    consecutive_bypass = 0;
                }

                if consecutive_bypass == ID_LEN {
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

        Ok(idcodes)
    }
}

fn reset2dr(mpsse: &FtdiMpsse, tck_mask: u8, tms_mask: u8) -> Result<(), FtdiError> {
    let direction = tck_mask | tms_mask;
    let mut cmd = MpsseCmdBuilder::new();
    cmd.set_gpio_lower(tck_mask, direction);
    for _ in 0..5 {
        cmd.set_gpio_lower(tms_mask, direction)
            .set_gpio_lower(tck_mask | tms_mask, direction);
    }
    cmd.set_gpio_lower(0, direction)
        .set_gpio_lower(tck_mask, direction)
        .set_gpio_lower(tms_mask, direction)
        .set_gpio_lower(tck_mask | tms_mask, direction)
        .set_gpio_lower(0, direction)
        .set_gpio_lower(tck_mask, direction)
        .set_gpio_lower(0, direction)
        .set_gpio_lower(tck_mask, direction);
    mpsse.exec(cmd)?;
    Ok(())
}
