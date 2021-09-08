//! Activate or deactivate digital audio interface
#![allow(clippy::new_without_default)]

use crate::Command;
use core::marker::PhantomData;

/// Power down configuration builder.
#[derive(Debug, Eq, PartialEq)]
pub struct ActiveControl {
    data: u16,
}

impl Copy for ActiveControl {}

impl Clone for ActiveControl {
    fn clone(&self) -> Self {
        *self
    }
}

/// Instanciate a builder for power down configuration.
pub fn active_control() -> ActiveControl {
    ActiveControl::new()
}

impl ActiveControl {
    fn new() -> Self {
        Self { data: 0b1001 << 9 }
    }
    ///Activate digital audio interface
    #[must_use]
    pub fn active(mut self) -> ActiveControl {
        self.data |= 0b1;
        self
    }
    ///Deactivate digital audio interface
    #[must_use]
    pub fn inactive(mut self) -> ActiveControl {
        self.data &= !(0b1);
        self
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}
