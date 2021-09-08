//! Analogue Audio Path configuration
#![allow(clippy::new_without_default)]

use crate::Command;
use core::marker::PhantomData;

#[path = "side_att_db.rs"]
mod side_att_db;
pub use side_att_db::*;

/// Analogue audio path configuration builder.
#[derive(Debug, Eq, PartialEq)]
pub struct AnalogueAudioPath {
    data: u16,
}

impl Copy for AnalogueAudioPath {}

impl Clone for AnalogueAudioPath {
    fn clone(&self) -> Self {
        *self
    }
}

/// Instanciate a builder for Analogue audio path configuration.
pub fn analogue_audio_path() -> AnalogueAudioPath {
    AnalogueAudioPath::new()
}

impl AnalogueAudioPath {
    fn new() -> Self {
        Self {
            data: 0b100 << 9 | 0b1010,
        }
    }
    pub fn micboost(self) -> Micboost {
        Micboost { cmd: self }
    }
    pub fn mutemic(self) -> Mutemic {
        Mutemic { cmd: self }
    }
    pub fn insel(self) -> Insel {
        Insel { cmd: self }
    }
    pub fn bypass(self) -> Bypass {
        Bypass { cmd: self }
    }
    pub fn dacsel(self) -> Dacsel {
        Dacsel { cmd: self }
    }
    pub fn sidetone(self) -> Sidetone {
        Sidetone { cmd: self }
    }
    pub fn sideatt(self) -> Sideatt {
        Sideatt { cmd: self }
    }
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

impl_toggle_writer!(Micboost, AnalogueAudioPath, 0);
impl_toggle_writer!(Mutemic, AnalogueAudioPath, 1);
impl_toggle_writer!(Bypass, AnalogueAudioPath, 3);
impl_toggle_writer!(Sidetone, AnalogueAudioPath, 5);

pub enum InselV {
    Line,
    Microphone,
}

pub struct Insel {
    cmd: AnalogueAudioPath,
}

impl Insel {
    impl_bit!(AnalogueAudioPath, 2);
    impl_set_bit!(AnalogueAudioPath, 2);
    impl_clear_bit!(AnalogueAudioPath, 2);
    impl_set_bit!(microphone, AnalogueAudioPath, 2);
    impl_clear_bit!(line, AnalogueAudioPath, 2);

    #[must_use]
    pub fn variant(self, value: InselV) -> AnalogueAudioPath {
        match value {
            InselV::Microphone => self.microphone(),
            InselV::Line => self.line(),
        }
    }
}

pub enum DacselV {
    Deselect,
    Select,
}

pub struct Dacsel {
    cmd: AnalogueAudioPath,
}

impl Dacsel {
    impl_bit!(AnalogueAudioPath, 4);
    impl_set_bit!(AnalogueAudioPath, 4);
    impl_clear_bit!(AnalogueAudioPath, 4);
    impl_set_bit!(select, AnalogueAudioPath, 4);
    impl_clear_bit!(deselect, AnalogueAudioPath, 4);

    #[must_use]
    pub fn variant(self, value: DacselV) -> AnalogueAudioPath {
        match value {
            DacselV::Deselect => self.deselect(),
            DacselV::Select => self.select(),
        }
    }
}

///Control attenuation of the mic input when directly connected to ouput.
pub struct Sideatt {
    cmd: AnalogueAudioPath,
}

impl Sideatt {
    impl_bits!(AnalogueAudioPath, 2, 6);
    ///Set attenuation from a dB representation.
    pub fn db(mut self, volume: SideAttdB) -> AnalogueAudioPath {
        let mask = !((!0) << 2) << 6;
        self.cmd.data = self.cmd.data & !mask | (volume.into_raw() as u16);
        self.cmd
    }
}
