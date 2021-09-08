//! Command for wm8731

use core::marker::PhantomData;

pub mod active_control;
pub mod analogue_audio_path;
pub mod digital_audio_interface;
pub mod digital_audio_path;
pub mod headphone_out;
pub mod line_in;
pub mod power_down;
pub mod sampling;

#[doc(inline)]
pub use active_control::active_control;
#[doc(inline)]
pub use analogue_audio_path::analogue_audio_path;
#[doc(inline)]
pub use digital_audio_interface::digital_audio_interface;
#[doc(inline)]
pub use digital_audio_path::digital_audio_path;
#[doc(inline)]
pub use headphone_out::{left_headphone_out, right_headphone_out};
#[doc(inline)]
pub use line_in::{left_line_in, right_line_in};
#[doc(inline)]
pub use power_down::power_down;
#[doc(inline)]
pub use reset::reset;
#[doc(inline)]
pub use sampling::sampling;
#[doc(inline)]
pub use sampling::sampling_with_mclk;

pub mod reset {
    //! Reset the device
    #![allow(clippy::new_without_default)]
    use crate::Command;
    use core::marker::PhantomData;
    /// Reset command builder.
    #[derive(Debug, Eq, PartialEq)]
    pub struct Reset {
        data: u16,
    }

    impl Copy for Reset {}

    impl Clone for Reset {
        fn clone(&self) -> Self {
            *self
        }
    }

    /// Instantiate a reset command builder.
    pub fn reset() -> Reset {
        Reset::new()
    }

    impl Reset {
        fn new() -> Self {
            Self {
                data: 0b110 << 9 | 0b1001_1111,
            }
        }
        pub fn into_command(self) -> Command<()> {
            Command::<()> {
                data: self.data,
                t: PhantomData::<()>,
            }
        }
    }
}

///Represent a command to send to the codec, that is register address and content to write in it.
#[derive(Debug, Eq, PartialEq)]
pub struct Command<T> {
    pub(crate) data: u16,
    t: PhantomData<T>,
}

impl<T> Copy for Command<T> {}

impl<T> Clone for Command<T> {
    fn clone(&self) -> Self {
        *self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn _should_compile() {
        left_headphone_out().hpvol().bits(0b111111).into_command();
    }
}
