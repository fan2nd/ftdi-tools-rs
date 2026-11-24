# FTDI
# Supported Chips
FT232H,FT2232D,FT2232H,FT4232H
# Supported Function
- GPIO
- SPI
- IIC
- Jtag
- SWD
- JtagDetect
- FTDI configuration EEPROM backup and programming

## Device discovery

Use `list_devices()?` to distinguish USB enumeration errors from an empty list.
Each result includes the USB descriptors, detected chip and its MPSSE channels.
Use `list_devices_with_ids(&[(vid, pid)])?` for boards with a custom USB identity.

## Configuration EEPROM

Run `cargo run --bin ftdi-eeprom -- list` and copy the USB port printed for
the target board. Select the EEPROM part fitted to that board. For example,
use these commands for a board fitted with a 93C56, replacing PORT with the
listed port and FILE with a binary configuration image.

```sh
ftdi-eeprom read PORT 93c56 backup.bin
ftdi-eeprom program PORT 93c56 FILE
ftdi-eeprom set-ids PORT 93c56 NEW_VID:NEW_PID
ftdi-eeprom --id NEW_VID:NEW_PID list
```

The commands above use the installed binary (`cargo install --path .`).
IDs are hexadecimal. The read command creates a new file and preserves raw
bytes even if the EEPROM is blank or its checksum is damaged. Programming
accepts a standard single configuration image for the same FTDI chip and
EEPROM part, writes changed words in order, and verifies the full region.
Reconnect the board to reload the configuration. If programming is interrupted,
retry with the original known-good image and the same physical part selection.

The supported configuration sizes are 128 bytes for 93C46 and 256 bytes for
93C56/93C66. The separate upper 93C66 user area is outside the backup.
For blank parts the size must come from the board or its schematic.
To edit channel modes, drive settings or USB strings, supply a binary image
built for the board. The in-place field editor currently supports VID/PID.
See the [libftdi EEPROM implementation](https://www.intra2net.com/en/developer/libftdi/documentation/ftdi_8c_source.html)
for the underlying format.

# Thanks
- [ftdi-embedded-hal](https://github.com/ftdi-rs/ftdi-embedded-hal)
- [ftdi-mpsse](https://github.com/ftdi-rs/ftdi-mpsse)
- [probe-rs](https://github.com/probe-rs/probe-rs/tree/master/probe-rs/src/probe/ftdi/ftdaye)
