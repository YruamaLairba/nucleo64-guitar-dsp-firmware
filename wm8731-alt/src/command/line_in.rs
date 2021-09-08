//! Line inputs configuration
#![allow(clippy::new_without_default)]

use super::Command;
use core::marker::PhantomData;

#[path = "in_vol_db.rs"]
mod in_vol_db;
pub use in_vol_db::*;

///Marker indicating left channel
pub struct Left;

///Marker indicating right channel
pub struct Right;

/// Line in configuration builder.
#[derive(Debug, Eq, PartialEq)]
pub struct LineIn<CHANNEL> {
    data: u16,
    channel: PhantomData<CHANNEL>,
}

impl<CHANNEL> Copy for LineIn<CHANNEL> {}

impl<CHANNEL> Clone for LineIn<CHANNEL> {
    fn clone(&self) -> Self {
        *self
    }
}

/// Left line in configuration builder.
pub type LeftLineIn = LineIn<Left>;

/// Right line in configuration builder.
pub type RightLineIn = LineIn<Right>;

/// Instanciate a builder for left line in configuration.
pub fn left_line_in() -> LeftLineIn {
    LeftLineIn::new()
}

/// Instanciate a builder for right line in configuration.
pub fn right_line_in() -> RightLineIn {
    RightLineIn::new()
}

impl LeftLineIn {
    fn new() -> Self {
        Self {
            data: 0b0_1001_0111,
            channel: PhantomData::<Left>,
        }
    }
}

impl RightLineIn {
    fn new() -> Self {
        Self {
            data: 0x1 << 9 | 0b0_1001_0111,
            channel: PhantomData::<Right>,
        }
    }
}

impl<CHANNEL> LineIn<CHANNEL> {
    pub fn invol(self) -> Invol<CHANNEL> {
        Invol { cmd: self }
    }
    pub fn inmute(self) -> Inmute<CHANNEL> {
        Inmute { cmd: self }
    }
    pub fn inboth(self) -> Inboth<CHANNEL> {
        Inboth { cmd: self }
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

///Writer of LINVOL or RINVOL fields. Control line input volume.
pub struct Invol<CHANNEL> {
    cmd: LineIn<CHANNEL>,
}

impl<CHANNEL> Invol<CHANNEL> {
    impl_bits!(LineIn<CHANNEL>, 5, 0);
    ///Set volume from a dB representation.
    pub fn db(mut self, volume: InVoldB) -> LineIn<CHANNEL> {
        let mask = !((!0) << 5);
        self.cmd.data = self.cmd.data & !mask | (volume.into_raw() as u16);
        self.cmd
    }
}

impl_toggle_writer!(Inmute<CHANNEL>, LineIn<CHANNEL>, 7);
impl_toggle_writer!(Inboth<CHANNEL>, LineIn<CHANNEL>, 8);

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn left_line_in_new() {
        let cmd = left_line_in().into_command();
        let expected = 0b0000_0000_1001_0111;
        assert!(
            cmd.data == expected,
            "Got {:#b},expected {:#b}",
            cmd.data,
            expected
        )
    }
    #[test]
    fn right_line_in_new() {
        let cmd = right_line_in();
        let expected = 0b0000_0010_1001_0111;
        assert!(
            cmd.data == expected,
            "Got {:#b},expected {:#b}",
            cmd.data,
            expected
        )
    }
    #[test]
    fn set_bits_dont_overwrite() {
        let cmd = right_line_in();
        //this should trigger a warning
        //cmd.invol().bits(0b1111_1111);
        let cmd = cmd.invol().bits(0b1111_1111);
        let expected = 0b0000_0010_1001_1111;
        assert!(
            cmd.data == expected,
            "Got {:#b},expected {:#b}",
            cmd.data,
            expected
        )
    }
}
