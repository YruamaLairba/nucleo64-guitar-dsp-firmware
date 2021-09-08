//! Handle communication details
use core::marker::PhantomData;
use embedded_hal::blocking::{i2c, spi};
use embedded_hal::digital::v2::OutputPin;

use crate::Command;

///Represent a frame sended through I2C or SPI interface.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct Frame {
    data: u16,
}

impl From<Frame> for [u8; 2] {
    ///Allow to convert frame to an array directly usable with SPI and I2C abstraction from embedded-hal.
    fn from(frame: Frame) -> [u8; 2] {
        frame.data.to_be_bytes()
    }
}

impl From<Frame> for [u16; 1] {
    ///Allow to convert frame to an array directly usable with 16 bit word SPI abstraction from embedded-hal.
    fn from(frame: Frame) -> [u16; 1] {
        [frame.data]
    }
}

impl From<Frame> for u16 {
    ///Allow to convert frame in u16.
    fn from(frame: Frame) -> u16 {
        frame.data
    }
}

impl<T> From<Command<T>> for Frame {
    ///Allow to convert frame to an array directly usable with SPI and I2C abstraction from embedded-hal.
    fn from(cmd: Command<T>) -> Frame {
        Frame { data: cmd.data }
    }
}

/// Serial Interface abstraction for the wm8731 generic driver.
pub trait WriteFrame {
    fn send(&mut self, frame: Frame);
}

/// I2C communication implementation using embedded-hal.
pub struct I2CInterface<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> I2CInterface<I2C>
where
    I2C: i2c::Write,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }
    pub fn release(self) -> I2C {
        self.i2c
    }
}

impl<I2C> WriteFrame for I2CInterface<I2C>
where
    I2C: i2c::Write,
{
    fn send(&mut self, frame: Frame) {
        let frame: [u8; 2] = frame.into();
        let _ = self.i2c.write(self.address, &frame);
    }
}

/// Generic blocking SPI communication implementation using embedded-hal.
pub struct SPIInterface<SPI, CS, W> {
    spi: SPI,
    cs: CS,
    w: PhantomData<W>,
}

impl<SPI, CS, W> SPIInterface<SPI, CS, W> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi,
            cs,
            w: PhantomData::<W>,
        }
    }
    pub fn release(self) -> SPI {
        self.spi
    }
}

/// 8 bits words SPI communication implementation using embedded-hal.
pub type SPIInterfaceU8<SPI, CS> = SPIInterface<SPI, CS, u8>;

/// 16 bits words SPI communication implementation using embedded-hal.
pub type SPIInterfaceU16<SPI, CS> = SPIInterface<SPI, CS, u16>;

impl<SPI, CS> WriteFrame for SPIInterfaceU8<SPI, CS>
where
    SPI: spi::Write<u8>,
    CS: OutputPin,
{
    fn send(&mut self, frame: Frame) {
        let frame: [u8; 2] = frame.into();
        let _ = self.cs.set_low();
        let _ = self.spi.write(&frame);
        let _ = self.cs.set_high();
    }
}

impl<SPI, CS> WriteFrame for SPIInterfaceU16<SPI, CS>
where
    SPI: spi::Write<u16>,
    CS: OutputPin,
{
    fn send(&mut self, frame: Frame) {
        let frame: [u16; 1] = frame.into();
        let _ = self.cs.set_low();
        let _ = self.spi.write(&frame);
        let _ = self.cs.set_high();
    }
}
