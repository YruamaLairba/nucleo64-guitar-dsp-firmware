//! Headphone ouputs configuration
#![allow(clippy::new_without_default)]

use super::Command;
use core::marker::PhantomData;

#[path = "hp_vol_db.rs"]
mod hp_vol_db;
pub use hp_vol_db::*;

///Marker indicating left channel
pub struct Left;

///Marker indicating right channel
pub struct Right;

///Headphone out configuration builder
#[derive(Debug, Eq, PartialEq)]
pub struct HeadphoneOut<CHANNEL> {
    data: u16,
    channel: PhantomData<CHANNEL>,
}

impl<CHANNEL> Copy for HeadphoneOut<CHANNEL> {}

impl<CHANNEL> Clone for HeadphoneOut<CHANNEL> {
    fn clone(&self) -> Self {
        *self
    }
}

///Marker indicating left headphone output concern
pub type LeftHeadphoneOut = HeadphoneOut<Left>;

///Marker indicating left headphone output concern
pub type RightHeadphoneOut = HeadphoneOut<Right>;

/// Instanciate a builder for left headphone output configuration.
pub fn left_headphone_out() -> LeftHeadphoneOut {
    LeftHeadphoneOut::new()
}

/// Instanciate a builder for right headphone output configuration.
pub fn right_headphone_out() -> RightHeadphoneOut {
    RightHeadphoneOut::new()
}

impl LeftHeadphoneOut {
    fn new() -> Self {
        Self {
            data: 0x2 << 9 | 0b0_0111_1001,
            channel: PhantomData::<Left>,
        }
    }
}

impl RightHeadphoneOut {
    fn new() -> Self {
        Self {
            data: 0x3 << 9 | 0b0_0111_1001,
            channel: PhantomData::<Right>,
        }
    }
}

impl<CHANNEL> HeadphoneOut<CHANNEL> {
    pub fn hpvol(self) -> Hpvol<CHANNEL> {
        Hpvol { cmd: self }
    }
    pub fn zcen(self) -> Zcen<CHANNEL> {
        Zcen { cmd: self }
    }
    pub fn hpboth(self) -> Hpboth<CHANNEL> {
        Hpboth { cmd: self }
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

///Writer of LHPVOL or RHPVOL fields. Control headphone output volume.
pub struct Hpvol<CHANNEL> {
    cmd: HeadphoneOut<CHANNEL>,
}

impl<CHANNEL> Hpvol<CHANNEL> {
    impl_bits!(HeadphoneOut<CHANNEL>, 7, 0);
    ///Set volume from a dB representation.
    pub fn db(mut self, volume: HpVoldB) -> HeadphoneOut<CHANNEL> {
        let mask = !((!0) << 7);
        self.cmd.data = self.cmd.data & !mask | (volume.into_raw() as u16);
        self.cmd
    }
}

impl_toggle_writer!(Zcen<CHANNEL>, HeadphoneOut<CHANNEL>, 7);
impl_toggle_writer!(Hpboth<CHANNEL>, HeadphoneOut<CHANNEL>, 8);
