//! Backup and program an FTDI configuration EEPROM.
use ftdi_tools::{
    eeprom::{EepromType, FtdiEeprom, set_usb_ids},
    list_devices_with_ids,
};
use std::{env, error::Error, fs, io::Write};

const HELP: &str = "ftdi-eeprom [--id VID:PID] list
ftdi-eeprom [--id VID:PID] read PORT 93c46|93c56|93c66 FILE
ftdi-eeprom [--id VID:PID] program PORT 93c46|93c56|93c66 FILE
ftdi-eeprom [--id VID:PID] set-ids PORT 93c46|93c56|93c66 NEW_VID:NEW_PID

Copy PORT from list. IDs are hexadecimal. read creates a new backup file.
program and set-ids write hardware and verify the configuration.";

type Result<T> = std::result::Result<T, Box<dyn Error>>;

fn ids(text: &str) -> Result<(u16, u16)> {
    let (vid, pid) = text.split_once(':').ok_or("expected hexadecimal VID:PID")?;
    Ok((u16::from_str_radix(vid, 16)?, u16::from_str_radix(pid, 16)?))
}

fn port(info: &nusb::DeviceInfo) -> String {
    let path: Vec<_> = info.port_chain().iter().map(u8::to_string).collect();
    format!("{}/{}", info.bus_id(), path.join("."))
}

fn main() -> Result<()> {
    let args: Vec<_> = env::args().skip(1).collect();
    let (extra, args) = match args.as_slice() {
        [flag, id, rest @ ..] if flag == "--id" => (vec![ids(id)?], rest),
        args => (Vec::new(), args),
    };
    if args.is_empty() || matches!(args, [s] if s == "--help" || s == "-h") {
        println!("{HELP}");
        return Ok(());
    }
    if matches!(args, [s] if s == "list") {
        for d in list_devices_with_ids(&extra)? {
            let usb = &d.usb_device;
            println!(
                "{}  {:04x}:{:04x}  {:?}  {:?}  serial={}",
                port(usb),
                usb.vendor_id(),
                usb.product_id(),
                d.chip_type,
                d.interface,
                usb.serial_number().unwrap_or("-")
            );
        }
        return Ok(());
    }
    let [command, target, part, arg] = args else {
        return Err(HELP.into());
    };
    if !matches!(command.as_str(), "read" | "program" | "set-ids") {
        return Err(HELP.into());
    }
    let memory = match part.as_str() {
        "93c46" => EepromType::C46,
        "93c56" => EepromType::C56,
        "93c66" => EepromType::C66,
        _ => return Err("expected 93c46, 93c56 or 93c66".into()),
    };
    let mut matches = list_devices_with_ids(&extra)?
        .into_iter()
        .filter(|d| port(&d.usb_device) == *target);
    let device = matches.next().ok_or("selected USB port not found")?;
    if matches.next().is_some() {
        return Err("ambiguous USB port".into());
    }

    let mut eeprom = FtdiEeprom::open(&device.usb_device, memory)?;
    let image = match command.as_str() {
        "read" => {
            let image = eeprom.read()?;
            fs::File::create_new(arg)?.write_all(&image)?;
            println!("Saved {} configuration bytes", image.len());
            return Ok(());
        }
        "program" => fs::read(arg)?,
        "set-ids" => {
            let (vid, pid) = ids(arg)?;
            let mut image = eeprom.read()?;
            set_usb_ids(device.chip_type, memory, &mut image, vid, pid)?;
            image
        }
        _ => unreachable!(),
    };
    let changed = eeprom.program(&image)?;
    println!(
        "Verified {} configuration bytes; wrote {changed} words. Reconnect the device to load the configuration.",
        image.len()
    );
    Ok(())
}
