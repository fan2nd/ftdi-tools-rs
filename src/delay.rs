//! Host delay adapter implementing embedded-hal DelayNs.

use eh1::delay::DelayNs;
use std::time::Duration;

pub struct Delay;
impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(Duration::from_nanos(ns as u64));
    }
    fn delay_us(&mut self, us: u32) {
        std::thread::sleep(Duration::from_micros(us as u64));
    }
    fn delay_ms(&mut self, ms: u32) {
        std::thread::sleep(Duration::from_millis(ms as u64));
    }
}
