//! FTDI USB transport context and read/write helpers.

use crate::{FtdiError, Interface};
use futures_lite::future::try_zip;
use nusb::{
    MaybeFuture,
    transfer::{Bulk, ControlOut, ControlType, In, Out, Recipient},
};
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
    handle: nusb::Interface,

    interface: Interface,
}

impl FtdiContext {
    pub(crate) fn new(handle: nusb::Interface, interface: Interface) -> Self {
        Self { handle, interface }
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
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index: self.interface.index(),
                    data: &[],
                },
                Duration::from_secs(1),
            )
            .wait()
            .map_err(std::io::Error::from)?;

        Ok(())
    }

    fn usb_reset(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_SIO: u16 = 0;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_SIO)
    }

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
        let mut ep = self
            .handle
            .endpoint::<Bulk, Out>(self.interface.write_ep())
            .map_err(std::io::Error::from)?;
        let len = data.len();
        ep.submit(data.into());
        let completion = ep.next_complete().await;
        completion.status.map_err(std::io::Error::from)?;
        if completion.actual_len != len {
            return Err(FtdiError::Other("Short USB bulk write"));
        }
        Ok(())
    }
    async fn async_read(&self, data: &mut [u8]) -> Result<(), FtdiError> {
        if data.is_empty() {
            return Ok(());
        }
        let mut ep = self
            .handle
            .endpoint::<Bulk, In>(self.interface.read_ep())
            .map_err(std::io::Error::from)?;
        let mut read_len = 0;
        while read_len < data.len() {
            ep.submit(ep.allocate(ep.max_packet_size()));
            let result = ep
                .next_complete()
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
            let read_buf = data
                .get_mut(read_len..read_len + response_data.len())
                .ok_or(FtdiError::Other("Excess USB response data"))?;
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
        try_zip(self.async_write(write), self.async_read(read)).await?;
        Ok(())
    }
}
