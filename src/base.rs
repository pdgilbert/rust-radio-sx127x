//! Basic HAL functions for communicating with the radio device
//!
//! This provides decoupling between embedded hal traits and the RF device implementation.
// Copyright 2019 Ryan Kurte

use core::convert::Infallible;
use core::fmt::Debug;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal::spi::SpiDevice;

/// HAL trait for radio interaction, may be generic over SPI or UART connections
pub trait Hal {
    type Error: Debug + 'static;

    /// Reset the device
    fn reset(&mut self) -> Result<(), Self::Error>;

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Self::Error>;

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error>;

    /// Delay for the specified time
    fn delay_ns(&mut self, ns: u32) -> Result<(), Self::Error>;

    /// Read from radio with prefix
    fn prefix_read(&mut self, prefix: &[u8], data: &mut [u8]) -> Result<(), Self::Error>;

    /// Write to radio with prefix
    fn prefix_write(&mut self, prefix: &[u8], data: &[u8]) -> Result<(), Self::Error>;

   /// Read from the specified register
   fn read_regs<'a>(
    &mut self,
    reg: u8,
    data: &mut [u8],
    ) -> Result<(), Self::Error> {
        // Setup register read
        let out_buf: [u8; 1] = [reg as u8 & 0x7F];
        self.wait_busy()?;
        let r = self
            .prefix_read(&out_buf, data)
            .map(|_| ())
            .map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Write to the specified register
    fn write_regs(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        // Setup register write
        let out_buf: [u8; 1] = [reg as u8 | 0x80];
        self.wait_busy()?;
        let r = self.prefix_write(&out_buf, data).map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Write to the specified buffer
    fn write_buff(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        // Setup fifo buffer write
        let out_buf: [u8; 1] = [0x00 | 0x80];
        self.wait_busy()?;
        let r = self.prefix_write(&out_buf, data).map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Read from the specified buffer
    fn read_buff<'a>(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        // Setup fifo buffer read
        let out_buf: [u8; 1] = [0x00];
        self.wait_busy()?;
        let r = self
            .prefix_read(&out_buf, data)
            .map(|_| ())
            .map_err(|e| e.into());
        self.wait_busy()?;
        r
    }

    /// Read a single u8 value from the specified register
    fn read_reg(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut incoming = [0u8; 1];
        self.read_regs(reg.into(), &mut incoming)?;
        Ok(incoming[0])
    }

    /// Write a single u8 value to the specified register
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.write_regs(reg.into(), &[value])?;
        Ok(())
    }

    /// Update the specified register with the provided value & mask
    fn update_reg(
        &mut self,
        reg: u8,
        mask: u8,
        value: u8,
    ) -> Result<u8, Self::Error> {
        let existing = self.read_reg(reg)?;
        let updated = (existing & !mask) | (value & mask);
        self.write_reg(reg, updated)?;
        Ok(updated)
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature="defmt", derive(defmt::Format))]
pub enum HalError<Spi, Pin, Delay> {
    Spi(Spi),
    Pin(Pin),
    Delay(Delay),
}

/// Spi base object defined interface for interacting with radio via SPI
pub struct Base <Spi: SpiDevice, Cs: OutputPin, Busy: InputPin, Ready: InputPin, Sdn: OutputPin, Delay: DelayNs> {
    pub spi: Spi,
    pub cs: Cs,
    pub busy: Busy,
    pub ready: Ready,
    pub sdn: Sdn,
    pub delay: Delay,
}

/// Implement HAL for base object
impl<Spi, Cs, Busy, Ready, Sdn, PinError, Delay> Hal for Base<Spi, Cs, Busy, Ready, Sdn, Delay>
where
    Spi: SpiDevice,
    <Spi as embedded_hal::spi::ErrorType>::Error : Debug + 'static,
    
    Cs: OutputPin<Error=PinError>,
    Busy: InputPin<Error=PinError>,
    Ready: InputPin<Error=PinError>,
    Sdn: OutputPin<Error=PinError>,
    PinError: Debug + 'static,

    Delay: DelayNs,
{
    type Error = HalError<<Spi as embedded_hal::spi::ErrorType>::Error, PinError, Infallible>;

    /// Reset the radio
    fn reset(&mut self) -> Result<(), Self::Error> {
        self.sdn.set_low().map_err(HalError::Pin)?;

        self.delay.delay_ms(1);

        self.sdn.set_high().map_err(HalError::Pin)?;

        self.delay.delay_ms(10);

        Ok(())
    }

    /// Wait on radio device busy
    fn wait_busy(&mut self) -> Result<(), Self::Error> {
        // TODO: suspiciously unimplemented?!
        Ok(())
    }

    /// Delay for the specified time
    fn delay_ms(&mut self, ms: u32) -> Result<(), Self::Error> {
        self.delay.delay_ms(ms);
        Ok(())
    }

    /// Delay for the specified time
    fn delay_ns(&mut self, ns: u32) -> Result<(), Self::Error> {
        self.delay.delay_ns(ns);
        Ok(())
    }

    /// Write data with prefix, asserting CS as required
    fn prefix_write(&mut self, prefix: &[u8], data: &[u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(HalError::Pin)?;

        let r = self.spi.write(prefix).map(|_| self.spi.write(data));

        self.cs.set_high().map_err(HalError::Pin)?;

        match r {
            Ok(Ok(_)) => Ok(()),
            Ok(Err(e)) | Err(e) => Err(HalError::Spi(e)),
        }
    }

    /// Read data with prefix, asserting CS as required
    fn prefix_read(&mut self, prefix: &[u8], data: &mut [u8]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(HalError::Pin)?;

        let r = self.spi.write(prefix).map(|_| self.spi.transfer_in_place(data));

        self.cs.set_high().map_err(HalError::Pin)?;

        match r {
            Ok(Ok(_)) => Ok(()),
            Ok(Err(e)) | Err(e) => Err(HalError::Spi(e)),
        }
    }
}
