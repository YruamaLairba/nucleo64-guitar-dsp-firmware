//! Command builder for sampling configuration.
//!
//! This module offer two style for building a command, a nice style using a master clock and a raw
//! style allowing more advance use.
//!
//! # Style with master clock
//! With this method, the builder is instantiated with a marker to indicate the internal master clock
//! frequency. Valid markers are :
//!  - `Mclk12M288` for a 12.288 MHz master clock
//!  - `Mclk18M432` for a 18.432 MHz master clock
//!  - `Mclk11M2896` for a 11.2896 MHz master clock
//!  - `Mclk16M9344` for a 16.9344 MHz master clock
//!  - `Mclk12M` for a 12MHz master clock, correspond to USB mode.
//!
//! You also don't write directly to USB/NORMAL, BOSR, or SR fields. Instead, you use a *virtual*
//! SampleRate field that do it for you.
//!
//!  ## Example
//! ```
//! # use wm8731_alt::command::sampling::*;
//! //instantiate the builder
//! let cmd = sampling_with_mclk(Mclk12M288);
//! //setup the sampling rate
//! let cmd = cmd.sample_rate().adc48k_dac48k();
//! //build the command
//! let cmd = cmd.into_command();
//! ```
//!
//! # Raw style
//! With this method, you write directly to USB/NORMAL, BOSR, and SR fields. This way is useful for
//! case not handled by the other method. Notably, the Sr field writer don't have very meaningful
//! method name, because same combination of USB/NORMAL, BOSR, and SR can produce different
//! sampling rate by just changing the master clock. Look the
//! [WAN0117](https://statics.cirrus.com/pubs/appNote/WAN0117.pdf) application notice for
//! advanced sampling rate selection.
//!
//! ## Example
//! ```
//! # use wm8731_alt::command::sampling::*;
//! //instantiate the builder
//! let cmd = sampling();
//! //normal mode operation
//! let cmd = cmd.usb_normal().normal();
//! //write bosr bit
//! let cmd = cmd.bosr().clear_bit();
//! //write sr field
//! let cmd = cmd.sr().sr_0b0000();
//! //build the command
//! let cmd = cmd.into_command();
//! ```
//!
//! # Safety and coherence
//! To guarantee safety and coherence, some manipulation are enforced or prohibited.
//!
//! When indicating a Master clock:
//!  - `sample_rate` need to be set explicitly.
//!  - available sample rate is Master Clock dependent.
//!
//! With the raw method:
//!  - if `usb_normal` or `bosr` are written, `sr` is invalidated and need to be rewritten.
//!  - available `sr` setting depends on `usb_normal` and `bosr` setting.
//!
//! ## Example of bad usage
//! Following example show incorrect usage and should not compile.
//! ```
//! # #[cfg(any())] //avoid some compilation error when testing doc
//! # {
//! # use wm8731_alt::command::sampling::*;
//! //error, sample rate require to be explicitly set
//! let cmd = sampling_with_mclk(Mclk12M288).into_command();
//! //error, this sampling rate setup is impossible with the current master clock
//! let cmd = sampling_with_mclk(Mclk12M288).sample_rate().adc44k1_dac44k1();
//! //error, change USB/Normal invalidate SR.
//! let cmd = sampling().usb_normal().usb().into_command();
//! //error, change BOSR invalidate SR.
//! let cmd = sampling().bosr().clear_bit().into_command();
//! //error, USB/NORMAL, BOSR, SR combination is invalid
//! let cmd =
//! sampling().usb_normal().usb().bosr().set_bit().sr().sr_0b0000();
//! # }
//! ```
//!
//!
#![allow(clippy::new_without_default)]

use crate::Command;
use core::marker::PhantomData;

pub mod state_marker {
    //! Markers to track state of the sampling builder.
    //!
    //! They are used with the sampling builder to provide coherent API and compile time safety check.

    /// Marker used to indicate Normal mode.
    pub struct Normal;
    /// Marker used to indicate USB mode.
    pub struct Usb;
    /// Marker used to indicate BOSR bit is set.
    pub struct BosrSet;
    /// Marker used to indicate BOSR bit is clear.
    pub struct BosrClear;
    /// Marker used to indicate Sr or SampleRate is valid.
    pub struct SrValid;
    /// Marker used to indicate Sr or SampleRate is not valid.
    ///
    /// `Sampling` configuration marked with this can not produce a command.
    pub struct SrInvalid;
}

use state_marker::*;

/// Builder for sampling command.
#[derive(Debug, Eq, PartialEq)]
pub struct Sampling<T> {
    data: u16,
    t: PhantomData<T>,
}

impl<T> Copy for Sampling<T> {}

impl<T> Clone for Sampling<T> {
    fn clone(&self) -> Self {
        *self
    }
}

//common to both method it's always safe to manipulate those fields
impl<T> Sampling<T> {
    pub fn clkidiv2(self) -> Clkidiv2<T> {
        Clkidiv2 { cmd: self }
    }
    pub fn clkodiv2(self) -> Clkodiv2<T> {
        Clkodiv2 { cmd: self }
    }
}

///Marker indicating use of 12.288Mhz internal master clock (normal mode).
pub struct Mclk12M288;
impl Mclk for Mclk12M288 {}
///Marker indicating use of 18.432Mhz internal master clock (normal mode).
pub struct Mclk18M432;
impl Mclk for Mclk18M432 {}
///Marker indicating use of 11.2896Mhz internal master clock (normal mode).
pub struct Mclk11M2896;
impl Mclk for Mclk11M2896 {}
///Marker indicating use of 16.9344Mhz internal master clock (normal mode).
pub struct Mclk16M9344;
impl Mclk for Mclk16M9344 {}
///Marker indicating use of 12Mhz internal master clock (USB mode).
pub struct Mclk12M;
impl Mclk for Mclk12M {}
/// Marker trait to say a marker correspond to a master clock value.
pub trait Mclk {}

/// Instantiate a command builder to set sampling configuration for a particular master clock.
pub fn sampling_with_mclk<MCLK>(_: MCLK) -> Sampling<(MCLK, SrInvalid)>
where
    MCLK: Mclk,
{
    Sampling::<(MCLK, SrInvalid)> {
        data: 0b1000 << 9,
        t: PhantomData::<(MCLK, SrInvalid)>,
    }
}

impl<MCLK, SR> Sampling<(MCLK, SR)>
where
    MCLK: Mclk,
{
    pub fn sample_rate(self) -> SampleRate<(MCLK, SR)> {
        SampleRate::<(MCLK, SR)> { cmd: self }
    }
}
/// Virtual field writer for more meaningful sampling rate setting.
///
/// This actually write USB/NORMAL, BOSR, and SR fields.
pub struct SampleRate<T> {
    cmd: Sampling<T>,
}

impl<MCLK, SR> SampleRate<(MCLK, SR)> {
    unsafe fn bits(mut self, value: u8) -> Sampling<(MCLK, SrValid)> {
        let mask = !((!0) << 6);
        self.cmd.data = self.cmd.data & !mask | (value as u16) << 2 & mask;
        Sampling::<(MCLK, SrValid)> {
            data: self.cmd.data,
            t: PhantomData::<(MCLK, SrValid)>,
        }
    }
}

impl<SR> SampleRate<(Mclk12M288, SR)> {
    ///Set 48khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc48k_dac48k(self) -> Sampling<(Mclk12M288, SrValid)> {
        unsafe { self.bits(0b000000) }
    }
    ///Set sampling rate of 48khz for ADC and 8khz for DAC.
    #[must_use]
    pub fn adc48k_dac8k(self) -> Sampling<(Mclk12M288, SrValid)> {
        unsafe { self.bits(0b000100) }
    }
    ///Set sampling rate of 8khz for ADC and 48khz for DAC.
    #[must_use]
    pub fn adc8k_dac48k(self) -> Sampling<(Mclk12M288, SrValid)> {
        unsafe { self.bits(0b001000) }
    }
    ///Set 8khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc8k_dac8k(self) -> Sampling<(Mclk12M288, SrValid)> {
        unsafe { self.bits(0b001100) }
    }
    ///Set 32khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc32k_dac32k(self) -> Sampling<(Mclk12M288, SrValid)> {
        unsafe { self.bits(0b011000) }
    }
    ///Set 96khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc96k_dac96k(self) -> Sampling<(Mclk12M288, SrValid)> {
        unsafe { self.bits(0b011100) }
    }
}

impl<SR> SampleRate<(Mclk18M432, SR)> {
    ///Set 48khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc48k_dac48k(self) -> Sampling<(Mclk18M432, SrValid)> {
        unsafe { self.bits(0b000010) }
    }
    ///Set sampling rate of 48khz for ADC and 8khz for DAC.
    #[must_use]
    pub fn adc48k_dac8k(self) -> Sampling<(Mclk18M432, SrValid)> {
        unsafe { self.bits(0b000110) }
    }
    ///Set sampling rate of 8khz for ADC and 48khz for DAC.
    #[must_use]
    pub fn adc8k_dac48k(self) -> Sampling<(Mclk18M432, SrValid)> {
        unsafe { self.bits(0b001010) }
    }
    ///Set 8khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc8k_dac8k(self) -> Sampling<(Mclk18M432, SrValid)> {
        unsafe { self.bits(0b001110) }
    }
    ///Set 32khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc32k_dac32k(self) -> Sampling<(Mclk18M432, SrValid)> {
        unsafe { self.bits(0b011010) }
    }
    ///Set 96khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc96k_dac96k(self) -> Sampling<(Mclk18M432, SrValid)> {
        unsafe { self.bits(0b011110) }
    }
}

impl<SR> SampleRate<(Mclk11M2896, SR)> {
    ///Set 44.1khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc44k1_dac44k1(self) -> Sampling<(Mclk11M2896, SrValid)> {
        unsafe { self.bits(0b100000) }
    }
    ///Set sampling rate of 44.1khz for ADC and approximatively 8khz for DAC.
    ///
    ///The actual DAC sampling rate is 8.018kHz
    #[must_use]
    pub fn adc44k1_dac8k(self) -> Sampling<(Mclk11M2896, SrValid)> {
        unsafe { self.bits(0b100100) }
    }
    ///Set sampling rate of approximatively 8khz for ADC and 44.1khz for DAC.
    ///
    ///The actual ADC sampling rate is 8.018kHz
    #[must_use]
    pub fn adc8k_dac44k1(self) -> Sampling<(Mclk11M2896, SrValid)> {
        unsafe { self.bits(0b101000) }
    }
    ///Set approximatively 8khz sampling rate for ADC and DAC.
    ///
    ///The actual sampling rate is 8.018kHz
    #[must_use]
    pub fn adc8k_dac8k(self) -> Sampling<(Mclk11M2896, SrValid)> {
        unsafe { self.bits(0b101100) }
    }
    ///Set 88.2khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc88k2_dac88k2(self) -> Sampling<(Mclk11M2896, SrValid)> {
        unsafe { self.bits(0b111100) }
    }
}

impl<SR> SampleRate<(Mclk16M9344, SR)> {
    ///Set 44.1khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc44k1_dac44k1(self) -> Sampling<(Mclk16M9344, SrValid)> {
        unsafe { self.bits(0b100010) }
    }
    ///Set sampling rate of 44.1khz for ADC and approximatively 8khz for DAC.
    ///
    ///The actual DAC sampling rate is 8.018kHz
    #[must_use]
    pub fn adc44k1_dac8k(self) -> Sampling<(Mclk16M9344, SrValid)> {
        unsafe { self.bits(0b100110) }
    }
    ///Set sampling rate of approximatively 8khz for ADC and 44.1khz for DAC.
    ///
    ///The actual ADC sampling rate is 8.018kHz
    #[must_use]
    pub fn adc8k_dac44k1(self) -> Sampling<(Mclk16M9344, SrValid)> {
        unsafe { self.bits(0b101010) }
    }
    ///Set approximatively 8khz sampling rate for ADC and DAC.
    ///
    ///The actual sampling rate is 8.018kHz
    #[must_use]
    pub fn adc8k_dac8k(self) -> Sampling<(Mclk16M9344, SrValid)> {
        unsafe { self.bits(0b101110) }
    }
    ///Set 88.2khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc88k2_dac88k2(self) -> Sampling<(Mclk16M9344, SrValid)> {
        unsafe { self.bits(0b111110) }
    }
}

impl<SR> SampleRate<(Mclk12M, SR)> {
    ///Set 48khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc48k_dac48k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b000001) }
    }
    ///Set approximatively 44.1khz sampling rate for ADC and DAC.
    ///
    ///The actual sampling rate is 44.118kHz.
    #[must_use]
    pub fn adc44k1_dac44k1(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b100011) }
    }
    ///Set sampling rate of 48khz for ADC and 8khz for DAC.
    #[must_use]
    pub fn adc48k_dac8k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b000101) }
    }
    ///Set sampling rate of approximatively 44.1khz for ADC and approximatively 8khz for DAC.
    ///
    ///The actual sampling rate are 44.118kHz for the ADC and 8.021kHz for the DAC.
    #[must_use]
    pub fn adc44k1_dac8k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b100111) }
    }
    ///Set sampling rate of 8khz for ADC and 48khz for DAC.
    #[must_use]
    pub fn adc8k_dac48k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b001001) }
    }
    ///Set sampling rate of approximatively 8khz for ADC and approximatively 44.1khz for DAC.
    ///
    ///The actual sampling rate are 8.021kHz for the ADC and 44.118kHz  for the DAC.
    #[must_use]
    pub fn adc8k_dac44k1(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b101011) }
    }
    ///Set 8khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc8k_dac8k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b001101) }
    }
    ///Set approximatively 8khz sampling rate for ADC and DAC.
    ///
    ///The actual sampling rate is 8.021kHz.
    #[must_use]
    pub fn adc8k_dac8k_bis(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b101111) }
    }
    ///Set 32khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc32k_dac32k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b011001) }
    }
    ///Set 96khz sampling rate for ADC and DAC.
    #[must_use]
    pub fn adc96k_dac96k(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b011101) }
    }
    ///Set approximatively 88.2kHz sampling rate for ADC and DAC.
    ///
    ///The actual sampling rate is 88.235kHz.
    #[must_use]
    pub fn adc88k2_dac88k2(self) -> Sampling<(Mclk12M, SrValid)> {
        unsafe { self.bits(0b111111) }
    }
}

//Once SampleRate have been explicitly set, a valid command can be instantiated
impl<MCLK> Sampling<(MCLK, SrValid)> {
    /// Instanciate a command
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

/// Instanciate a command builder for sampling configuration.
pub fn sampling() -> Sampling<(Normal, BosrClear, SrValid)> {
    Sampling::<(Normal, BosrClear, SrValid)>::new()
}

impl Sampling<(Normal, BosrClear, SrValid)> {
    #[allow(clippy::identity_op)]
    fn new() -> Self {
        Self {
            data: 0b1000 << 9 | 0b0000_0000,
            t: PhantomData::<(Normal, BosrClear, SrValid)>,
        }
    }
}

//Once sr have been explicitly set, a valid command can be instantiated
impl<MODE, BOSR> Sampling<(MODE, BOSR, SrValid)> {
    /// Instanciate a command
    pub fn into_command(self) -> Command<()> {
        Command::<()> {
            data: self.data,
            t: PhantomData::<()>,
        }
    }
}

//field accessible in raw mode
impl<MODE, BOSR, SR> Sampling<(MODE, BOSR, SR)> {
    pub fn usb_normal(self) -> UsbNormal<(MODE, BOSR, SR)> {
        UsbNormal { cmd: self }
    }
    pub fn bosr(self) -> Bosr<(MODE, BOSR, SR)> {
        Bosr { cmd: self }
    }
    pub fn sr(self) -> Sr<(MODE, BOSR, SR)> {
        Sr { cmd: self }
    }
}

/// Field writer. Allow to select USB or Normal mode. Invalidate `Sr` field.
pub struct UsbNormal<T> {
    cmd: Sampling<T>,
}

impl<MODE, BOSR, SR> UsbNormal<(MODE, BOSR, SR)> {
    #[must_use]
    pub fn clear_bit(mut self) -> Sampling<(Normal, BOSR, SrInvalid)> {
        self.cmd.data &= !(0b1 << 0);
        Sampling::<(Normal, BOSR, SrInvalid)> {
            data: self.cmd.data,
            t: PhantomData::<(Normal, BOSR, SrInvalid)>,
        }
    }
    #[must_use]
    pub fn set_bit(mut self) -> Sampling<(Usb, BOSR, SrInvalid)> {
        self.cmd.data |= 0b1 << 0;
        Sampling::<(Usb, BOSR, SrInvalid)> {
            data: self.cmd.data,
            t: PhantomData::<(Usb, BOSR, SrInvalid)>,
        }
    }
    #[must_use]
    pub fn normal(mut self) -> Sampling<(Normal, BOSR, SrInvalid)> {
        self.cmd.data &= !(0b1 << 0);
        Sampling::<(Normal, BOSR, SrInvalid)> {
            data: self.cmd.data,
            t: PhantomData::<(Normal, BOSR, SrInvalid)>,
        }
    }
    #[must_use]
    pub fn usb(mut self) -> Sampling<(Usb, BOSR, SrInvalid)> {
        self.cmd.data |= 0b1 << 0;
        Sampling::<(Usb, BOSR, SrInvalid)> {
            data: self.cmd.data,
            t: PhantomData::<(Usb, BOSR, SrInvalid)>,
        }
    }
}

/// Field writer. Select the Base Over-Sampling Rate. Invalidate `Sr` field.
pub struct Bosr<T> {
    cmd: Sampling<T>,
}

impl<MODE, BOSR, SR> Bosr<(MODE, BOSR, SR)> {
    #[must_use]
    pub fn clear_bit(mut self) -> Sampling<(MODE, BosrClear, SrInvalid)> {
        self.cmd.data &= !(0b1 << 1);
        Sampling::<(MODE, BosrClear, SrInvalid)> {
            data: self.cmd.data,
            t: PhantomData::<(MODE, BosrClear, SrInvalid)>,
        }
    }
    #[must_use]
    pub fn set_bit(mut self) -> Sampling<(MODE, BosrSet, SrInvalid)> {
        self.cmd.data |= 0b1 << 1;
        Sampling::<(MODE, BosrSet, SrInvalid)> {
            data: self.cmd.data,
            t: PhantomData::<(MODE, BosrSet, SrInvalid)>,
        }
    }
}

/// Field writer. Allow to write raw bits into the sr field.
pub struct Sr<T> {
    cmd: Sampling<T>,
}

impl<MODE, BOSR, SR> Sr<(MODE, BOSR, SR)> {
    //impl_bits!(unsafe, Sampling<T>, 4, 2);
    /// Set the field with raw bits.
    ///
    /// # Safety
    ///
    /// This is unsafe because it assume valid bits combination that may actually not. Please read
    /// the datasheet to know what are the valid combinations.
    pub unsafe fn bits(mut self, value: u8) -> Sampling<(MODE, BOSR, SrValid)> {
        let mask = !((!0) << 4) << 2;
        self.cmd.data = self.cmd.data & !mask | (value as u16) << 2 & mask;
        Sampling::<(MODE, BOSR, SrValid)> {
            data: self.cmd.data,
            t: PhantomData::<(MODE, BOSR, SrValid)>,
        }
    }
}

impl<BOSR, SR> Sr<(Normal, BOSR, SR)> {
    #[must_use]
    pub fn sr_0b0000(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b0000) }
    }
    #[must_use]
    pub fn sr_0b0001(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b0001) }
    }
    #[must_use]
    pub fn sr_0b0010(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b0010) }
    }
    #[must_use]
    pub fn sr_0b0011(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b0011) }
    }
    #[must_use]
    pub fn sr_0b0110(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b0110) }
    }
    #[must_use]
    pub fn sr_0b0111(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b0111) }
    }
    #[must_use]
    pub fn sr_0b1000(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b1000) }
    }
    #[must_use]
    pub fn sr_0b1001(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b1001) }
    }
    #[must_use]
    pub fn sr_0b1010(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b1010) }
    }
    #[must_use]
    pub fn sr_0b1011(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b1011) }
    }
    #[must_use]
    pub fn sr_0b1111(self) -> Sampling<(Normal, BOSR, SrValid)> {
        unsafe { self.bits(0b1111) }
    }
}

impl<SR> Sr<(Usb, BosrClear, SR)> {
    #[must_use]
    pub fn sr_0b0000(self) -> Sampling<(Usb, BosrClear, SrValid)> {
        unsafe { self.bits(0b0000) }
    }
    #[must_use]
    pub fn sr_0b0001(self) -> Sampling<(Usb, BosrClear, SrValid)> {
        unsafe { self.bits(0b0001) }
    }
    #[must_use]
    pub fn sr_0b0010(self) -> Sampling<(Usb, BosrClear, SrValid)> {
        unsafe { self.bits(0b0010) }
    }
    #[must_use]
    pub fn sr_0b0011(self) -> Sampling<(Usb, BosrClear, SrValid)> {
        unsafe { self.bits(0b0011) }
    }
    #[must_use]
    pub fn sr_0b0110(self) -> Sampling<(Usb, BosrClear, SrValid)> {
        unsafe { self.bits(0b0110) }
    }
    #[must_use]
    pub fn sr_0b0111(self) -> Sampling<(Usb, BosrClear, SrValid)> {
        unsafe { self.bits(0b0111) }
    }
}

impl<SR> Sr<(Usb, BosrSet, SR)> {
    #[must_use]
    pub fn sr_0b1000(self) -> Sampling<(Usb, BosrSet, SrValid)> {
        unsafe { self.bits(0b1000) }
    }
    #[must_use]
    pub fn sr_0b1001(self) -> Sampling<(Usb, BosrSet, SrValid)> {
        unsafe { self.bits(0b1001) }
    }
    #[must_use]
    pub fn sr_0b1010(self) -> Sampling<(Usb, BosrSet, SrValid)> {
        unsafe { self.bits(0b1010) }
    }
    #[must_use]
    pub fn sr_0b1011(self) -> Sampling<(Usb, BosrSet, SrValid)> {
        unsafe { self.bits(0b1011) }
    }
    #[must_use]
    pub fn sr_0b1111(self) -> Sampling<(Usb, BosrSet, SrValid)> {
        unsafe { self.bits(0b1111) }
    }
}

impl_toggle_writer!(Clkidiv2<T>, Sampling<T>, 6);
impl_toggle_writer!(Clkodiv2<T>, Sampling<T>, 7);

#[cfg(test)]
mod tests {
    use super::*;
    // all() to compile, any() to not compile
    #[cfg(all())]
    fn _should_compile() {
        let _ = sampling_with_mclk(Mclk12M288)
            .sample_rate()
            .adc48k_dac48k()
            .into_command();
        let new_cmd = sampling();
        //default is valid
        new_cmd.into_command();
        //setting sr from default is valid
        new_cmd.sr().sr_0b0000().into_command();
    }
    // all() to compile, any() to not compile
    #[cfg(any())]
    fn _should_compile_warn() {
        let new_cmd = sampling();
        //should warn, you may think you change the command but this is not the case
        new_cmd.usb_normal().normal();

        let cmd = sampling_with_mclk(Mclk12M288);
        //should warn, you may think you change the command but this is not the case
        cmd.sample_rate().adc48k_dac48k();
    }
    // all() to compile, any() to not compile
    #[cfg(any())]
    fn _should_compile_error() {
        //error, when specifying mclk, Sampling rate default value is undefined.
        sampling_with_mclk(Mclk11M2896).into_command();
        //error, invalid combinations of clock and sample rate.
        sampling_with_mclk(Mclk11M2896)
            .sample_rate()
            .adc48k_dac48k();
        sampling_with_mclk(Mclk16M9344)
            .sample_rate()
            .adc96k_dac96k();
        sampling_with_mclk(Mclk12M288)
            .sample_rate()
            .adc44k1_dac44k1();
        sampling_with_mclk(Mclk18M432)
            .sample_rate()
            .adc88k1_dac88k1();

        let new_cmd = sampling();
        //error, can't build the command, setting USB/Normal invalidate sr.
        let _ = new_cmd.usb_normal().normal().into_command();
        //error, can't build the command, setting BOSR invalidate sr.
        let _ = new_cmd.bosr().clear_bit().into_command();
        //error, cannot set this sr value with this bosr value
        let _ = new_cmd
            .usb_normal()
            .usb()
            .bosr()
            .clear_bit()
            .sr()
            .sr_0b1111();
        //error, cannot set this sr value with this bosr value
        let _ = new_cmd.usb_normal().usb().bosr().set_bit().sr().sr_0b0000();
    }
}
