//! Digital Audio Path configuration
#![allow(clippy::new_without_default)]

use crate::Command;
use core::marker::PhantomData;

/// Digital audio path configuration builder.
#[derive(Debug, Eq, PartialEq)]
pub struct DigitalAudioPath {
    data: u16,
}

impl Copy for DigitalAudioPath {}

impl Clone for DigitalAudioPath {
    fn clone(&self) -> Self {
        *self
    }
}

/// Instanciate a builder for Digital audio path configuration.
pub fn digital_audio_path() -> DigitalAudioPath {
    DigitalAudioPath::new()
}

impl DigitalAudioPath {
    fn new() -> Self {
        Self {
            data: 0b101 << 9 | 0b1000,
        }
    }
    pub fn adchpd(self) -> Adchpd {
        Adchpd { cmd: self }
    }
    pub fn deemp(self) -> Deemp {
        Deemp { cmd: self }
    }
    pub fn dacmu(self) -> Dacmu {
        Dacmu { cmd: self }
    }
    pub fn hpor(self) -> Hpor {
        Hpor { cmd: self }
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

impl_toggle_writer!(Adchpd, DigitalAudioPath, 0);
impl_toggle_writer!(Dacmu, DigitalAudioPath, 3);

pub enum DeempV {
    Disable = 0b00,
    F32k = 0b01,
    F44k1 = 0b10,
    F48k = 0b11,
}

pub struct Deemp {
    cmd: DigitalAudioPath,
}

impl Deemp {
    impl_bits!(DigitalAudioPath, 2, 1);

    #[must_use]
    pub fn variant(self, value: DeempV) -> DigitalAudioPath {
        match value {
            DeempV::Disable => self.bits(0b00),
            DeempV::F32k => self.bits(0b01),
            DeempV::F44k1 => self.bits(0b10),
            DeempV::F48k => self.bits(0b11),
        }
    }
    pub fn disable(self) -> DigitalAudioPath {
        self.bits(0b00)
    }
    pub fn f32k(self) -> DigitalAudioPath {
        self.bits(0b01)
    }
    pub fn f44k1(self) -> DigitalAudioPath {
        self.bits(0b10)
    }
    pub fn f48k(self) -> DigitalAudioPath {
        self.bits(0b11)
    }
}

pub enum HporV {
    ClearOffset,
    StoreOffset,
}

pub struct Hpor {
    cmd: DigitalAudioPath,
}

impl Hpor {
    impl_bit!(DigitalAudioPath, 4);
    impl_clear_bit!(DigitalAudioPath, 4);
    impl_set_bit!(DigitalAudioPath, 4);
    impl_clear_bit!(clear_offset, DigitalAudioPath, 4);
    impl_set_bit!(store_offset, DigitalAudioPath, 4);

    #[must_use]
    pub fn variant(self, value: HporV) -> DigitalAudioPath {
        match value {
            HporV::ClearOffset => self.clear_offset(),
            HporV::StoreOffset => self.store_offset(),
        }
    }
}
