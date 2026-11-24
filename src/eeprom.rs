//! FTDI external configuration EEPROM access via vendor requests on endpoint zero.
//! Images contain little-endian words, including the final checksum word.

use crate::{ChipType, FtdiError};
use nusb::{
    MaybeFuture,
    transfer::{ControlIn, ControlOut, ControlType, Recipient},
};
use std::{io, time::Duration};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EepromType {
    C46,
    C56,
    C66,
}

impl EepromType {
    /// Configuration area in bytes. The upper user area of a 93C66 is separate.
    pub fn config_size(self) -> usize {
        match self {
            Self::C46 => 128,
            Self::C56 | Self::C66 => 256,
        }
    }

    fn marker(self) -> u8 {
        match self {
            Self::C46 => 0x46,
            Self::C56 => 0x56,
            Self::C66 => 0x66,
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum EepromError {
    #[error(transparent)]
    Device(#[from] FtdiError),
    #[error("EEPROM {operation} at word {address:#04x}: {source}")]
    Transfer {
        operation: &'static str,
        address: u16,
        source: io::Error,
    },
    #[error("invalid EEPROM configuration: {0}")]
    Image(&'static str),
    #[error("EEPROM verification failed at word {0:#04x}")]
    Verify(usize),
}

fn checksum(data: &[u8]) -> u16 {
    data.as_chunks::<2>()
        .0
        .iter()
        .fold(0xaaaa_u16, |sum, word| {
            (sum ^ u16::from_le_bytes(*word)).rotate_left(1)
        })
}

fn layout(chip: ChipType) -> Result<(u16, usize), EepromError> {
    Ok(match chip {
        ChipType::FT2232D => (0x500, 0x14),
        ChipType::FT2232H => (0x700, 0x18),
        ChipType::FT4232H => (0x800, 0x18),
        ChipType::FT232H => (0x900, 0x1e),
        _ => return Err(FtdiError::UnsupportedChip(chip).into()),
    })
}

pub fn validate_image(chip: ChipType, memory: EepromType, image: &[u8]) -> Result<(), EepromError> {
    let (version, marker) = layout(chip)?;
    if image.len() != memory.config_size() {
        return Err(EepromError::Image("wrong configuration area length"));
    }
    if image[6..8] != version.to_le_bytes() {
        return Err(EepromError::Image("image targets a different FTDI chip"));
    }
    if image[marker] != memory.marker() {
        return Err(EepromError::Image(
            "EEPROM type marker does not match the selected part",
        ));
    }
    let (body, stored) = image.split_at(image.len() - 2);
    if checksum(body).to_le_bytes() != stored {
        return Err(EepromError::Image(
            "incorrect checksum or unsupported mirrored layout",
        ));
    }
    if image.len() == 256 && image[..128] == image[128..] {
        return Err(EepromError::Image(
            "mirrored EEPROM layouts are unsupported",
        ));
    }
    Ok(())
}

/// Edit a validated standard image offline, preserving other configuration words.
pub fn set_usb_ids(
    chip: ChipType,
    memory: EepromType,
    image: &mut [u8],
    vid: u16,
    pid: u16,
) -> Result<(), EepromError> {
    validate_image(chip, memory, image)?;
    if vid == 0 || vid == 0xffff {
        return Err(EepromError::Image("reserved USB vendor ID"));
    }
    image[2..4].copy_from_slice(&vid.to_le_bytes());
    image[4..6].copy_from_slice(&pid.to_le_bytes());
    let (body, stored) = image.split_at_mut(image.len() - 2);
    stored.copy_from_slice(&checksum(body).to_le_bytes());
    Ok(())
}

pub struct FtdiEeprom {
    interface: nusb::Interface,
    chip: ChipType,
    memory: EepromType,
}

const TIMEOUT: Duration = Duration::from_secs(1);

impl FtdiEeprom {
    /// Select the actual EEPROM part fitted to the board, including for blank parts.
    /// Claims channel A without resetting the device or enabling MPSSE.
    pub fn open(info: &nusb::DeviceInfo, memory: EepromType) -> Result<Self, EepromError> {
        let chip = ChipType::from_device(info)?;
        let device = info.open().wait().map_err(|e| FtdiError::Usb(e.into()))?;
        let interface = device
            .detach_and_claim_interface(0)
            .wait()
            .map_err(|e| FtdiError::Usb(e.into()))?;
        Ok(Self {
            interface,
            chip,
            memory,
        })
    }

    /// Return the raw configuration region even for a blank or damaged EEPROM.
    pub fn read(&self) -> Result<Vec<u8>, EepromError> {
        let mut bytes = Vec::with_capacity(self.memory.config_size());
        for index in 0..(self.memory.config_size() / 2) as u16 {
            let error = |source| EepromError::Transfer {
                operation: "read",
                address: index,
                source,
            };
            let data = self
                .interface
                .control_in(
                    ControlIn {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Device,
                        request: 0x90,
                        value: 0,
                        index,
                        length: 2,
                    },
                    TIMEOUT,
                )
                .wait()
                .map_err(|e| error(e.into()))?;
            if data.len() != 2 {
                return Err(error(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "expected a 16-bit word",
                )));
            }
            bytes.extend_from_slice(&data);
        }
        Ok(bytes)
    }

    /// Program changed words in address order, then verify the complete configuration.
    /// A transfer failure can leave a partial image. Retry with the same known-good image.
    pub fn program(&mut self, image: &[u8]) -> Result<usize, EepromError> {
        validate_image(self.chip, self.memory, image)?;
        let current = self.read()?;
        let (_, marker) = layout(self.chip)?;
        if matches!(current[marker], 0x46 | 0x56 | 0x66) && current[marker] != self.memory.marker()
        {
            return Err(EepromError::Image(
                "target EEPROM type differs from the selected part",
            ));
        }
        let changed = write_changes(&current, image, |index, value| {
            self.interface
                .control_out(
                    ControlOut {
                        control_type: ControlType::Vendor,
                        recipient: Recipient::Device,
                        request: 0x91,
                        value,
                        index,
                        data: &[],
                    },
                    TIMEOUT,
                )
                .wait()
                .map_err(|e| EepromError::Transfer {
                    operation: "write",
                    address: index,
                    source: e.into(),
                })
        })?;
        let actual = self.read()?;
        if let Some(offset) = actual.iter().zip(image).position(|(a, b)| a != b) {
            return Err(EepromError::Verify(offset / 2));
        }
        Ok(changed)
    }
}

fn write_changes(
    current: &[u8],
    image: &[u8],
    mut write: impl FnMut(u16, u16) -> Result<(), EepromError>,
) -> Result<usize, EepromError> {
    let mut changed = 0;
    for (index, (old, new)) in current
        .as_chunks::<2>()
        .0
        .iter()
        .zip(image.as_chunks::<2>().0.iter())
        .enumerate()
    {
        if old != new {
            write(index as u16, u16::from_le_bytes(*new))?;
            changed += 1;
        }
    }
    Ok(changed)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(memory: EepromType) -> Vec<u8> {
        let mut bytes = vec![0; memory.config_size()];
        bytes[6..8].copy_from_slice(&0x700u16.to_le_bytes());
        bytes[0x18] = memory.marker();
        let (body, end) = bytes.split_at_mut(memory.config_size() - 2);
        end.copy_from_slice(&checksum(body).to_le_bytes());
        bytes
    }

    #[test]
    fn known_checksum_uses_little_endian_words_and_rotation() {
        assert_eq!(checksum(&[0x34, 0x12, 0x78, 0x56]), 0x4e8a);
    }

    #[test]
    fn reject_bad_images_before_editing() {
        let mut bytes = fixture(EepromType::C56);
        assert!(validate_image(ChipType::FT232H, EepromType::C56, &bytes).is_err());
        assert!(validate_image(ChipType::FT2232H, EepromType::C66, &bytes).is_err());
        assert!(validate_image(ChipType::FT2232H, EepromType::C46, &bytes).is_err());
        bytes[42] ^= 1;
        let original = bytes.clone();
        assert!(set_usb_ids(ChipType::FT2232H, EepromType::C56, &mut bytes, 1, 2).is_err());
        assert_eq!(bytes, original);
    }

    #[test]
    fn program_changes_checksum_last_and_retry_partial_write() {
        for memory in [EepromType::C46, EepromType::C56, EepromType::C66] {
            let mut target = fixture(memory);
            let mut desired = target.clone();
            set_usb_ids(ChipType::FT2232H, memory, &mut desired, 0x403, 0x6010).unwrap();
            let prior = target.clone();
            let result = write_changes(&prior, &desired, |index, word| {
                if index == 2 {
                    return Err(EepromError::Verify(2));
                }
                let pos = usize::from(index) * 2;
                target[pos..pos + 2].copy_from_slice(&word.to_le_bytes());
                Ok(())
            });
            assert!(result.is_err());
            let mut writes = Vec::new();
            write_changes(&target, &desired, |index, _| {
                writes.push(index);
                Ok(())
            })
            .unwrap();
            assert_eq!(writes, [2, (memory.config_size() / 2 - 1) as u16]);
            assert_eq!(
                write_changes(&desired, &desired, |_, _| panic!("redundant write")).unwrap(),
                0
            );
        }
    }
}
