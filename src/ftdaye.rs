use crate::{FtdiError, Interface};
use futures_lite::future::zip;
use nusb::transfer::{Control, ControlType, Recipient, RequestBuffer};
use std::time::Duration;

#[repr(C)]
#[expect(unused)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum BitMode {
    Reset = 0,
    Bitbang = 1,
    Mpsse = 2,
    SyncBb = 4,
    Mcu = 8,
    Opto = 16,
    Cbus = 32,
    SyncFf = 64,
    Ft1284 = 128,
}

pub(crate) struct FtdiContext {
    /// USB device handle
    handle: nusb::Interface,
    /// FTDI device interface
    interface: Interface,
    max_packet_size: usize,
}

impl FtdiContext {
    pub(crate) fn new(
        handle: nusb::Interface,
        interface: Interface,
        max_packet_size: usize,
    ) -> Self {
        Self {
            handle,
            interface,
            max_packet_size,
        }
    }
    pub(crate) fn into_mpsse(mut self, mask: u8) -> Result<Self, FtdiError> {
        self.usb_reset()?;
        self.usb_purge_buffers()?;
        self.set_latency_timer(16)?;
        self.set_bitmode(mask, BitMode::Mpsse)?;
        Ok(self)
    }
    fn sio_write(&mut self, request: u8, value: u16) -> Result<(), FtdiError> {
        self.handle
            .control_out_blocking(
                Control {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index: self.interface.index(),
                },
                &[],
                Duration::from_secs(1),
            )
            .map_err(std::io::Error::from)?;

        Ok(())
    }

    fn usb_reset(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_SIO: u16 = 0;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_SIO)
    }

    /// Clears the write buffer on the chip.
    fn usb_purge_tx_buffer(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_PURGE_TX: u16 = 2;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_PURGE_TX)
    }

    fn usb_purge_rx_buffer(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_PURGE_RX: u16 = 1;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_PURGE_RX)?;

        Ok(())
    }

    fn usb_purge_buffers(&mut self) -> Result<(), FtdiError> {
        self.usb_purge_tx_buffer()?;
        self.usb_purge_rx_buffer()?;

        Ok(())
    }

    fn set_latency_timer(&mut self, value: u8) -> Result<(), FtdiError> {
        const SIO_SET_LATENCY_TIMER_REQUEST: u8 = 0x09;

        self.sio_write(SIO_SET_LATENCY_TIMER_REQUEST, value as u16)
    }

    fn set_bitmode(&mut self, bitmask: u8, mode: BitMode) -> Result<(), FtdiError> {
        const SIO_SET_BITMODE_REQUEST: u8 = 0x0B;

        self.sio_write(
            SIO_SET_BITMODE_REQUEST,
            u16::from_le_bytes([bitmask, mode as u8]),
        )?;

        Ok(())
    }
    pub(crate) async fn async_write(&self, data: Vec<u8>) -> Result<(), FtdiError> {
        self.handle
            .bulk_out(self.interface.write_ep(), data)
            .await
            .into_result()
            .map_err(std::io::Error::from)?;
        Ok(())
    }
    pub(crate) async fn async_read(&self, data: &mut [u8]) -> Result<(), FtdiError> {
        let mut read_len = 0;
        while read_len < data.len() {
            let result = self
                .handle
                .bulk_in(
                    self.interface.read_ep(),
                    RequestBuffer::new(self.max_packet_size),
                )
                .await
                .into_result()
                .map_err(std::io::Error::from)?;
            if result.len() < 2 {
                return Err(FtdiError::Other("Usb bulkin length not correct"));
            }
            let (response_status, response_data) = result.split_at(2);
            if response_status[0] == 0xFA {
                return Err(FtdiError::BadMpsseCommand(response_status[1]));
            }
            let (_, read_buf) = data.split_at_mut(read_len);
            let (read_buf, _) = read_buf.split_at_mut(response_data.len());
            read_buf.copy_from_slice(response_data);
            read_len += response_data.len()
        }
        Ok(())
    }
    pub(crate) async fn async_write_read(
        &self,
        write: Vec<u8>,
        read: &mut [u8],
    ) -> Result<(), FtdiError> {
        let (write_result, read_result) = zip(self.async_write(write), self.async_read(read)).await;
        write_result?;
        read_result
    }
}
