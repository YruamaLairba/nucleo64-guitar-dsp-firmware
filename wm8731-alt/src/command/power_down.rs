//! Power down configuration
#![allow(clippy::new_without_default)]

use crate::Command;
use core::marker::PhantomData;

/// Power down configuration builder.
#[derive(Debug, Eq, PartialEq)]
pub struct PowerDown {
    data: u16,
}

impl Copy for PowerDown {}

impl Clone for PowerDown {
    fn clone(&self) -> Self {
        *self
    }
}

/// Instanciate a builder for power down configuration.
pub fn power_down() -> PowerDown {
    PowerDown::new()
}

impl PowerDown {
    fn new() -> Self {
        Self {
            data: 0b110 << 9 | 0b1001_1111,
        }
    }
    pub fn lineinpd(self) -> Lineinpd {
        Lineinpd { cmd: self }
    }
    pub fn micpd(self) -> Micpd {
        Micpd { cmd: self }
    }
    pub fn adcpd(self) -> Adcpd {
        Adcpd { cmd: self }
    }
    pub fn dacpd(self) -> Dacpd {
        Dacpd { cmd: self }
    }
    pub fn outpd(self) -> Outpd {
        Outpd { cmd: self }
    }
    pub fn oscpd(self) -> Oscpd {
        Oscpd { cmd: self }
    }
    pub fn clkoutpd(self) -> Clkoutpd {
        Clkoutpd { cmd: self }
    }
    pub fn poweroff(self) -> Poweroff {
        Poweroff { cmd: self }
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

impl_toggle_writer!(Lineinpd, PowerDown, 0);
impl_toggle_writer!(Micpd, PowerDown, 1);
impl_toggle_writer!(Adcpd, PowerDown, 2);
impl_toggle_writer!(Dacpd, PowerDown, 3);
impl_toggle_writer!(Outpd, PowerDown, 4);
impl_toggle_writer!(Oscpd, PowerDown, 5);
impl_toggle_writer!(Clkoutpd, PowerDown, 6);
impl_toggle_writer!(Poweroff, PowerDown, 7);
