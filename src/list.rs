//! FTDI USB device discovery using cached descriptors, without claiming interfaces.

use nusb::{DeviceInfo, MaybeFuture};

use crate::{ChipType, FtdiError, Interface};

const KNOWN_IDS: &[(u16, u16)] = &[
    (0x0403, 0x6010),
    (0x0403, 0x6011),
    (0x0403, 0x6014),
    (0x15ba, 0x0003),
    (0x15ba, 0x0004),
    (0x15ba, 0x002a),
    (0x15ba, 0x002b),
];

pub struct FtdiDeviceInfo {
    pub usb_device: DeviceInfo,
    pub interface: &'static [Interface],
    pub chip_type: ChipType,
}

pub fn list_devices() -> Result<Vec<FtdiDeviceInfo>, FtdiError> {
    list_devices_with_ids(&[])
}

/// Include devices configured with one of the supplied custom VID/PID pairs.
pub fn list_devices_with_ids(additional: &[(u16, u16)]) -> Result<Vec<FtdiDeviceInfo>, FtdiError> {
    let devices = nusb::list_devices().wait().map_err(std::io::Error::from)?;
    Ok(devices
        .filter_map(|usb_device| {
            let id = (usb_device.vendor_id(), usb_device.product_id());
            if !KNOWN_IDS.contains(&id) && !additional.contains(&id) {
                return None;
            }
            let chip_type = ChipType::from_device(&usb_device).ok()?;
            Some(FtdiDeviceInfo {
                usb_device,
                interface: chip_type.mpsse_list(),
                chip_type,
            })
        })
        .collect())
}

/// Compatibility entry point. Use `list_devices` to distinguish failures from an empty result.
pub fn list_all_device() -> Vec<FtdiDeviceInfo> {
    list_devices().unwrap_or_else(|err| {
        log::warn!("FTDI enumeration failed: {err}");
        Vec::new()
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shared_product_id_has_different_mpsse_interfaces() {
        assert_eq!(
            ChipType::from_version(0x500, None).unwrap().mpsse_list(),
            &[Interface::A]
        );
        assert_eq!(
            ChipType::from_version(0x700, None).unwrap().mpsse_list(),
            &[Interface::A, Interface::B]
        );
        assert!(ChipType::from_version(0xffff, None).is_err());
    }
}
