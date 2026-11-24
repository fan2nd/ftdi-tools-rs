//! Example: jtag detect

use std::time::Instant;

use ftdi_tools::{
    jtag::{JtagDetectTdi, JtagDetectTdo},
    list_all_device,
    mpsse::FtdiMpsse,
};
use itertools::Itertools;

fn main() -> anyhow::Result<()> {
    env_logger::init();

    let now = Instant::now();

    let devices = list_all_device();

    assert!(!devices.is_empty(), "Not found Ftdi devices");

    let mpsse = FtdiMpsse::open(&devices[0].usb_device, devices[0].interface[0])?;

    let mut notdi: Vec<_> = Vec::new();

    let mut jtag = JtagDetectTdo::new(mpsse);
    for couple in (0..8).permutations(2) {
        let tck = couple[0];
        let tms = couple[1];

        jtag.set_pins(tck, tms);

        jtag.scan()?
            .into_iter()
            .for_each(|tdo| notdi.push((tck, tms, tdo)));
    }

    let mut jtag = JtagDetectTdi::new(jtag);
    for (tck, tms, tdo) in notdi {
        for tdi in 0..8 {
            if tdi == tck || tdi == tms || tdi == tdo {
                continue;
            }

            jtag.set_pins(tck, tdi, tdo, tms);

            let ids_scan1 = jtag.scan_with(true)?;
            let ids_scan0 = jtag.scan_with(false)?;

            if ids_scan0.len() - ids_scan1.len() == 32 {
                println!("!!!!!! Pins:tck[{tck}],tdi[{tdi}],tdo[{tdo}],tms[{tms}]");

                println!("!!!!!! Found Devices:{ids_scan1:x?}");
            }
        }
    }

    println!("Finish Detect Using {:?}", now.elapsed());
    Ok(())
}
