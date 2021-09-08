//! Digital Audio Path configuration
#![allow(clippy::new_without_default)]

use crate::Command;
use core::marker::PhantomData;

/// builder for digital audio interface configuration
#[derive(Debug, Eq, PartialEq)]
pub struct DigitalAudioInterface {
    data: u16,
}

impl Copy for DigitalAudioInterface {}

impl Clone for DigitalAudioInterface {
    fn clone(&self) -> Self {
        *self
    }
}

/// Instanciate a builder for digital audio interface configuration.
pub fn digital_audio_interface() -> DigitalAudioInterface {
    DigitalAudioInterface::new()
}

impl DigitalAudioInterface {
    fn new() -> Self {
        Self {
            data: 0b111 << 9 | 0b1010,
        }
    }
    pub fn format(self) -> Format {
        Format { cmd: self }
    }
    pub fn iwl(self) -> Iwl {
        Iwl { cmd: self }
    }
    pub fn lrp(self) -> Lrp {
        Lrp { cmd: self }
    }
    pub fn lrswap(self) -> Lrswap {
        Lrswap { cmd: self }
    }
    pub fn ms(self) -> Ms {
        Ms { cmd: self }
    }
    pub fn bclkinv(self) -> Bclkinv {
        Bclkinv { cmd: self }
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

pub enum FormatV {
    Dsp = 0b11,
    I2s = 0b10,
    LeftJustified = 0b01,
    RigthJustified = 0b00,
}

pub struct Format {
    cmd: DigitalAudioInterface,
}

impl Format {
    impl_bits!(DigitalAudioInterface, 2, 0);

    #[must_use]
    pub fn variant(self, value: FormatV) -> DigitalAudioInterface {
        match value {
            FormatV::Dsp => self.bits(0b11),
            FormatV::I2s => self.bits(0b10),
            FormatV::LeftJustified => self.bits(0b01),
            FormatV::RigthJustified => self.bits(0b00),
        }
    }

    #[must_use]
    pub fn dsp(self) -> DigitalAudioInterface {
        self.bits(0b11)
    }
    #[must_use]
    pub fn i2s(self) -> DigitalAudioInterface {
        self.bits(0b10)
    }
    #[must_use]
    pub fn left_justified(self) -> DigitalAudioInterface {
        self.bits(0b01)
    }
    #[must_use]
    pub fn right_justified(self) -> DigitalAudioInterface {
        self.bits(0b00)
    }
}

pub enum IwlV {
    Iwl32bits = 0b11,
    Iwl24bits = 0b10,
    Iwl20bits = 0b01,
    Iwl16bits = 0b00,
}

pub struct Iwl {
    cmd: DigitalAudioInterface,
}

impl Iwl {
    impl_bits!(DigitalAudioInterface, 2, 2);

    #[must_use]
    pub fn variant(self, value: IwlV) -> DigitalAudioInterface {
        match value {
            IwlV::Iwl32bits => self.bits(0b11),
            IwlV::Iwl24bits => self.bits(0b10),
            IwlV::Iwl20bits => self.bits(0b01),
            IwlV::Iwl16bits => self.bits(0b00),
        }
    }
    #[must_use]
    pub fn iwl_32_bits(self) -> DigitalAudioInterface {
        self.bits(0b11)
    }
    #[must_use]
    pub fn iwl_24_bits(self) -> DigitalAudioInterface {
        self.bits(0b10)
    }
    #[must_use]
    pub fn iwl_20_bits(self) -> DigitalAudioInterface {
        self.bits(0b01)
    }
    #[must_use]
    pub fn iwl_16_bits(self) -> DigitalAudioInterface {
        self.bits(0b00)
    }
}

pub struct Lrp {
    cmd: DigitalAudioInterface,
}

impl Lrp {
    impl_bit!(DigitalAudioInterface, 4);
    impl_clear_bit!(DigitalAudioInterface, 4);
    impl_set_bit!(DigitalAudioInterface, 4);
}

impl_toggle_writer!(Lrswap, DigitalAudioInterface, 5);

pub enum MsV {
    Master = 0b1,
    Slave = 0b0,
}

pub struct Ms {
    cmd: DigitalAudioInterface,
}

impl Ms {
    impl_bit!(DigitalAudioInterface, 6);
    impl_clear_bit!(DigitalAudioInterface, 6);
    impl_set_bit!(DigitalAudioInterface, 6);
    impl_clear_bit!(slave, DigitalAudioInterface, 6);
    impl_set_bit!(master, DigitalAudioInterface, 6);

    #[must_use]
    pub fn variant(self, value: MsV) -> DigitalAudioInterface {
        match value {
            MsV::Slave => self.slave(),
            MsV::Master => self.master(),
        }
    }
}

impl_toggle_writer!(Bclkinv, DigitalAudioInterface, 7);
