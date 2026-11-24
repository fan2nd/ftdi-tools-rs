//! Low-level FTDI MPSSE command encoder.

#[repr(u8)]
#[derive(Debug, Copy, Clone)]
enum MpsseCmd {
    SetDataBitsLowbyte = 0x80,

    GetDataBitsLowbyte = 0x81,

    SetDataBitsHighbyte = 0x82,

    GetDataBitsHighbyte = 0x83,

    EnableLoopback = 0x84,

    DisableLoopback = 0x85,

    SetClockFrequency = 0x86,

    SendImmediate = 0x87,

    _WaitOnIOHigh = 0x88,

    _WaitOnIOLow = 0x89,

    DisableClockDivideBy5 = 0x8A,

    EnableClockDivideBy5 = 0x8B,

    Enable3PhaseClocking = 0x8C,

    Disable3PhaseClocking = 0x8D,

    EnableAdaptiveClocking = 0x96,

    DisableAdaptiveClocking = 0x97,

    _EnableDriveOnlyZero = 0x9E,
}

#[bitfield_struct::bitfield(u8, order = Lsb)]
struct MpsseShiftCmd {
    is_tdi_neg_write: bool,
    #[bits(default = true)]
    is_bit_mode: bool,
    is_tdo_neg_read: bool,
    #[bits(default = true)]
    is_lsb: bool,
    is_tdi_write: bool,
    is_tdo_read: bool,
    #[bits(default = false)]
    is_tms_write: bool,
    #[bits(default = false)]
    _const_0: bool,
}
impl MpsseShiftCmd {
    fn shift(
        tck_init_value: bool,
        is_bit_mode: bool,
        is_lsb: bool,
        is_tdi_write: bool,
        is_tdo_read: bool,
    ) -> u8 {
        assert!(
            is_tdi_write | is_tdo_read,
            "tdi_write and tdo_read can not be false tonight"
        );
        MpsseShiftCmd::new()
            .with_is_tdi_neg_write((!tck_init_value) && is_tdi_write)
            .with_is_bit_mode(is_bit_mode)
            .with_is_tdo_neg_read(tck_init_value && is_tdo_read)
            .with_is_lsb(is_lsb)
            .with_is_tdi_write(is_tdi_write)
            .with_is_tdo_read(is_tdo_read)
            .into()
    }
    fn _tms_shift(tck_init_value: bool, tdo_neg_read: bool, tdo_read: bool) -> u8 {
        MpsseShiftCmd::new()
            .with_is_tdi_neg_write(!tck_init_value)
            .with_is_tdo_neg_read(tdo_neg_read && tdo_read)
            .with_is_tdo_read(tdo_read)
            .with_is_tms_write(true)
            .into()
    }
    fn tms_shift(tdo_read: bool) -> u8 {
        Self::_tms_shift(false, false, tdo_read)
    }
}

const MAX_BYTES_SHIFT: usize = 65536;
const MAX_BITS_SHIFT: usize = 8;
const MAX_TMS_SHIFT: usize = 7;
#[derive(Default)]
pub(crate) struct MpsseCmdBuilder {
    cmd: Vec<u8>,
    read_len: usize,
}
impl MpsseCmdBuilder {
    pub(crate) fn new() -> MpsseCmdBuilder {
        Default::default()
    }

    pub(crate) fn append(&mut self, other: Self) {
        self.cmd.extend(other.cmd);
        self.read_len += other.read_len;
    }

    pub(crate) fn transfer_bytes(&mut self, clock: bool, lsb: bool, read_len: usize, write: &[u8]) {
        let common = read_len.min(write.len());
        self.shift_bytes(clock, lsb, &write[..common])
            .shift_bytes_out(clock, lsb, &write[common..])
            .shift_bytes_in(clock, lsb, read_len - common);
    }

    pub(crate) fn destruct(mut self) -> (Vec<u8>, Vec<u8>) {
        self.send_immediate();
        (self.cmd, vec![0; self.read_len])
    }

    pub(crate) fn set_clock(&mut self, divisor: u16, clk_div_by5: Option<bool>) -> &mut Self {
        match clk_div_by5 {
            Some(true) => self.cmd.push(MpsseCmd::EnableClockDivideBy5 as u8),
            Some(false) => self.cmd.push(MpsseCmd::DisableClockDivideBy5 as u8),
            None => {}
        };
        self.cmd.extend_from_slice(&[
            MpsseCmd::SetClockFrequency as u8,
            (divisor & 0xFF) as u8,
            ((divisor >> 8) & 0xFF) as u8,
        ]);
        self
    }

    pub(crate) fn enable_loopback(&mut self, state: bool) -> &mut Self {
        if state {
            self.cmd.push(MpsseCmd::EnableLoopback as u8);
        } else {
            self.cmd.push(MpsseCmd::DisableLoopback as u8);
        }
        self
    }

    pub(crate) fn enable_3phase_data_clocking(&mut self, state: bool) -> &mut Self {
        if state {
            self.cmd.push(MpsseCmd::Enable3PhaseClocking as u8);
        } else {
            self.cmd.push(MpsseCmd::Disable3PhaseClocking as u8);
        }
        self
    }

    pub(crate) fn enable_adaptive_clocking(&mut self, state: bool) -> &mut Self {
        if state {
            self.cmd.push(MpsseCmd::EnableAdaptiveClocking as u8);
        } else {
            self.cmd.push(MpsseCmd::DisableAdaptiveClocking as u8);
        }
        self
    }

    pub(crate) fn set_gpio_lower(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsLowbyte as u8, state, direction]);
        self
    }

    pub(crate) fn set_gpio_upper(&mut self, state: u8, direction: u8) -> &mut Self {
        self.cmd
            .extend_from_slice(&[MpsseCmd::SetDataBitsHighbyte as u8, state, direction]);
        self
    }

    pub(crate) fn gpio_lower(&mut self) -> &mut Self {
        self.read_len += 1;
        self.cmd.push(MpsseCmd::GetDataBitsLowbyte as u8);
        self
    }

    pub(crate) fn gpio_upper(&mut self) -> &mut Self {
        self.read_len += 1;
        self.cmd.push(MpsseCmd::GetDataBitsHighbyte as u8);
        self
    }

    fn send_immediate(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::SendImmediate as u8);
        self
    }

    pub(crate) fn _wait_on_io_high(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::_WaitOnIOHigh as u8);
        self
    }

    pub(crate) fn _wait_on_io_low(&mut self) -> &mut Self {
        self.cmd.push(MpsseCmd::_WaitOnIOLow as u8);
        self
    }

    pub(crate) fn shift_bytes_out(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        for slice in data.chunks(MAX_BYTES_SHIFT) {
            self.shift_bytes_out_limited(tck_init_value, is_lsb, slice);
        }
        self
    }
    fn shift_bytes_out_limited(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        let mut len = data.len();
        if len == 0 {
            return self;
        }
        assert!(
            len <= MAX_BYTES_SHIFT,
            "data length should be less than {MAX_BYTES_SHIFT}"
        );
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, true, false),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self.cmd.extend_from_slice(data);
        self
    }

    pub(crate) fn shift_bytes_in(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        mut len: usize,
    ) -> &mut Self {
        while len >= MAX_BYTES_SHIFT {
            self.shift_bytes_in_limited(tck_init_value, is_lsb, MAX_BYTES_SHIFT);
            len -= MAX_BYTES_SHIFT;
        }
        self.shift_bytes_in_limited(tck_init_value, is_lsb, len);
        self
    }
    fn shift_bytes_in_limited(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        mut len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(
            len <= MAX_BYTES_SHIFT,
            "data length should be less than {MAX_BYTES_SHIFT}"
        );
        self.read_len += len;
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, false, true),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self
    }

    pub(crate) fn shift_bytes(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        for slice in data.chunks(MAX_BYTES_SHIFT) {
            self.shift_bytes_limited(tck_init_value, is_lsb, slice);
        }
        self
    }
    fn shift_bytes_limited(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: &[u8],
    ) -> &mut Self {
        let mut len = data.len();
        if len == 0 {
            return self;
        }
        assert!(
            len <= MAX_BYTES_SHIFT,
            "data length should be less than {MAX_BYTES_SHIFT}"
        );
        self.read_len += len;
        len -= 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, false, is_lsb, true, true),
            (len & 0xFF) as u8,
            ((len >> 8) & 0xFF) as u8,
        ]);
        self.cmd.extend_from_slice(data);
        self
    }

    pub(crate) fn shift_bits_out(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: u8,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be less than {MAX_BITS_SHIFT}");
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, true, false),
            (len - 1) as u8,
            data,
        ]);
        self
    }

    pub(crate) fn shift_bits_in(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be less than {MAX_BITS_SHIFT}");
        self.read_len += 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, false, true),
            (len - 1) as u8,
        ]);
        self
    }

    pub(crate) fn shift_bits(
        &mut self,
        tck_init_value: bool,
        is_lsb: bool,
        data: u8,
        len: usize,
    ) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 8, "data length should be less than {MAX_BITS_SHIFT}");

        self.read_len += 1;
        self.cmd.extend_from_slice(&[
            MpsseShiftCmd::shift(tck_init_value, true, is_lsb, true, true),
            (len - 1) as u8,
            data,
        ]);
        self
    }

    pub(crate) fn clock_tms_out(&mut self, tdi: bool, data: u8, len: usize) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 7, "data length should be less than {MAX_TMS_SHIFT}");
        let data = if tdi { data | 0x80 } else { data };
        self.cmd
            .extend_from_slice(&[MpsseShiftCmd::tms_shift(false), (len - 1) as u8, data]);
        self
    }

    pub(crate) fn clock_tms(&mut self, tdi: bool, data: u8, len: usize) -> &mut Self {
        if len == 0 {
            return self;
        }
        assert!(len <= 7, "data length should be less than {MAX_TMS_SHIFT}");
        self.read_len += 1;
        let data = if tdi { data | 0x80 } else { data };
        self.cmd
            .extend_from_slice(&[MpsseShiftCmd::tms_shift(true), (len - 1) as u8, data]);
        self
    }
}
#[cfg(test)]
mod test {
    use super::MpsseShiftCmd;
    #[test]
    fn mpsse_shift_cmd_write_box_test() {
        assert_eq!(
            0x10u8,
            MpsseShiftCmd::shift(true, false, false, true, false)
        );
        assert_eq!(
            0x11u8,
            MpsseShiftCmd::shift(false, false, false, true, false)
        );
        assert_eq!(0x12u8, MpsseShiftCmd::shift(true, true, false, true, false));
        assert_eq!(
            0x13u8,
            MpsseShiftCmd::shift(false, true, false, true, false)
        );

        assert_eq!(
            0x20u8,
            MpsseShiftCmd::shift(false, false, false, false, true)
        );
        assert_eq!(
            0x24u8,
            MpsseShiftCmd::shift(true, false, false, false, true)
        );
        assert_eq!(
            0x22u8,
            MpsseShiftCmd::shift(false, true, false, false, true)
        );
        assert_eq!(0x26u8, MpsseShiftCmd::shift(true, true, false, false, true));

        assert_eq!(
            0x31u8,
            MpsseShiftCmd::shift(false, false, false, true, true)
        );
        assert_eq!(0x34u8, MpsseShiftCmd::shift(true, false, false, true, true));
        assert_eq!(0x33u8, MpsseShiftCmd::shift(false, true, false, true, true));
        assert_eq!(0x36u8, MpsseShiftCmd::shift(true, true, false, true, true));

        assert_eq!(0x18u8, MpsseShiftCmd::shift(true, false, true, true, false));
        assert_eq!(
            0x19u8,
            MpsseShiftCmd::shift(false, false, true, true, false)
        );
        assert_eq!(0x1au8, MpsseShiftCmd::shift(true, true, true, true, false));
        assert_eq!(0x1bu8, MpsseShiftCmd::shift(false, true, true, true, false));

        assert_eq!(
            0x28u8,
            MpsseShiftCmd::shift(false, false, true, false, true)
        );
        assert_eq!(0x2cu8, MpsseShiftCmd::shift(true, false, true, false, true));
        assert_eq!(0x2au8, MpsseShiftCmd::shift(false, true, true, false, true));
        assert_eq!(0x2eu8, MpsseShiftCmd::shift(true, true, true, false, true));

        assert_eq!(0x39u8, MpsseShiftCmd::shift(false, false, true, true, true));
        assert_eq!(0x3cu8, MpsseShiftCmd::shift(true, false, true, true, true));
        assert_eq!(0x3bu8, MpsseShiftCmd::shift(false, true, true, true, true));
        assert_eq!(0x3eu8, MpsseShiftCmd::shift(true, true, true, true, true));

        assert_eq!(0x4au8, MpsseShiftCmd::_tms_shift(true, false, false));
        assert_eq!(0x4au8, MpsseShiftCmd::_tms_shift(true, true, false));
        assert_eq!(0x4bu8, MpsseShiftCmd::_tms_shift(false, false, false));
        assert_eq!(0x4bu8, MpsseShiftCmd::tms_shift(false));
        assert_eq!(0x4bu8, MpsseShiftCmd::_tms_shift(false, true, false));
        assert_eq!(0x6au8, MpsseShiftCmd::_tms_shift(true, false, true));
        assert_eq!(0x6bu8, MpsseShiftCmd::_tms_shift(false, false, true));
        assert_eq!(0x6bu8, MpsseShiftCmd::tms_shift(true));
        assert_eq!(0x6eu8, MpsseShiftCmd::_tms_shift(true, true, true));
        assert_eq!(0x6fu8, MpsseShiftCmd::_tms_shift(false, true, true));
    }
    #[test]
    fn mpsse_shift_cmd_black_box_test() {
        use std::panic;
        let mut cmd_set = Vec::new();
        let permutations: Vec<_> = (0..32)
            .map(|i| {
                [
                    i & (1 << 0) != 0,
                    i & (1 << 1) != 0,
                    i & (1 << 2) != 0,
                    i & (1 << 3) != 0,
                    i & (1 << 4) != 0,
                ]
            })
            .collect();
        for permutation in permutations.iter() {
            let result: Result<u8, _> = panic::catch_unwind(|| {
                MpsseShiftCmd::shift(
                    permutation[0],
                    permutation[1],
                    permutation[2],
                    permutation[3],
                    permutation[4],
                )
            });
            if let Ok(x) = result {
                cmd_set.push(x);
            }
        }
        for permutation in permutations[0..8].iter() {
            let result: Result<u8, _> = panic::catch_unwind(|| {
                MpsseShiftCmd::_tms_shift(permutation[0], permutation[1], permutation[2])
            });
            if let Ok(x) = result {
                cmd_set.push(x);
            }
        }
        cmd_set.sort();
        cmd_set.dedup();
        assert_eq!(
            cmd_set,
            [
                0x10, 0x11, 0x12, 0x13, 0x18, 0x19, 0x1a, 0x1b, 0x20, 0x22, 0x24, 0x26, 0x28, 0x2a,
                0x2c, 0x2e, 0x31, 0x33, 0x34, 0x36, 0x39, 0x3b, 0x3c, 0x3e, 0x4a, 0x4b, 0x6a, 0x6b,
                0x6e, 0x6f
            ]
        )
    }
}
