use core::fmt;

///Error returned when trying to scale a value into SideAttdB.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum SideAttdBScaleError {
    ///The given range is null.
    NullRange,
    ///The given value is outside the given range.
    OutOfRange,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
/// Mic to output attenuation in dB. Beware, greater value mean greater attenuation, ie lower
/// volume.
pub struct SideAttdB {
    inner: u8,
}
impl SideAttdB {
    ///Instanciate an `SideAttdB` from it's underlaying representation.
    ///
    ///# Safety
    ///The raw value must be in between `0b00` and `0b11`.
    pub const unsafe fn from_raw_unchecked(raw: u8) -> Self {
        Self { inner: raw }
    }
    ///Instanciate an `SideAttdB` from it's underlaying representation.
    ///
    ///The raw value is saturated to fit in the range.
    pub const fn from_raw(raw: u8) -> Self {
        if raw > SideAttdB::MAX.inner {
            SideAttdB::MAX
        } else {
            Self { inner: raw }
        }
    }
    ///Return the raw underlaying representation
    pub const fn into_raw(self) -> u8 {
        self.inner
    }
    /// Scale a value into a SideAttdB. This function output an error when the input range is null or
    /// when the input is outside the range
    pub fn from_scaled(
        low_limit: i16,
        high_limit: i16,
        input: i16,
    ) -> Result<Self, SideAttdBScaleError> {
        if low_limit == high_limit {
            return Err(SideAttdBScaleError::NullRange);
        } else if (low_limit < high_limit && (input < low_limit || input > high_limit))
            || (low_limit > high_limit && (input > low_limit || input < high_limit))
        {
            return Err(SideAttdBScaleError::OutOfRange);
        }

        const MIN: u8 = SideAttdB::MIN.inner;
        const MAX: u8 = SideAttdB::MAX.inner;
        let r1 = high_limit - low_limit;
        let r2 = MAX - MIN;
        let res = (((input - low_limit) * r2 as i16) + r1 / 2) / (r1) + MIN as i16;
        let res = unsafe { SideAttdB::from_raw_unchecked(res as _) };
        Ok(res)
    }

    ///Increase the value by one step. Saturated to `SideAttdB::MAX`.
    pub fn increase(&mut self) {
        if self.inner < SideAttdB::MAX.inner {
            self.inner += 1;
        }
    }

    ///Decrease the value by one step. Saturated to `SideAttdB::MIN`.
    pub fn decrease(&mut self) {
        if self.inner > SideAttdB::MIN.inner {
            self.inner -= 1;
        }
    }
    ///Represent a value of -6dB
    pub const N6DB: SideAttdB = unsafe { SideAttdB::from_raw_unchecked(0b00) };
    ///Represent a value of -9dB
    pub const N9DB: SideAttdB = unsafe { SideAttdB::from_raw_unchecked(0b01) };
    ///Represent a value of -12dB
    pub const N12DB: SideAttdB = unsafe { SideAttdB::from_raw_unchecked(0b10) };
    ///Represent a value of -15dB
    pub const N15DB: SideAttdB = unsafe { SideAttdB::from_raw_unchecked(0b11) };
    ///Minimum attenuation
    pub const MIN: SideAttdB = SideAttdB::N6DB;
    ///Maximum attenuation
    pub const MAX: SideAttdB = SideAttdB::N15DB;
}
impl fmt::Display for SideAttdB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let fmt = match self.inner {
            0b00 => "-6",
            0b01 => "-9",
            0b10 => "-12",
            0b11 => "-15",
            _ => unreachable!(),
        };
        write!(f, "{}dB", fmt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn scale_test() {
        let db = SideAttdB::from_scaled(0, 255, 0).unwrap().inner;
        let expected = SideAttdB::MIN.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = SideAttdB::from_scaled(0, 255, 255).unwrap().inner;
        let expected = SideAttdB::MAX.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = SideAttdB::from_scaled(255, 0, 0).unwrap().inner;
        let expected = SideAttdB::MAX.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = SideAttdB::from_scaled(255, 0, 255).unwrap().inner;
        let expected = SideAttdB::MIN.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = SideAttdB::from_scaled(128, 128, 128).unwrap_err();
        let expected = SideAttdBScaleError::NullRange;
        assert!(db == expected, "Got {:?},expected {:?}", db, expected);
        let db = SideAttdB::from_scaled(0, 127, 128).unwrap_err();
        let expected = SideAttdBScaleError::OutOfRange;
        assert!(db == expected, "Got {:?},expected {:?}", db, expected);
    }
    #[test]
    fn increase_decrease_saturation_test() {
        let mut test = SideAttdB::MAX;
        test.increase();
        assert!(
            test == SideAttdB::MAX,
            "Got {}, expected {}",
            test,
            SideAttdB::MAX
        );
        let mut test = SideAttdB::MIN;
        test.decrease();
        assert!(
            test == SideAttdB::MIN,
            "Got {}, expected {}",
            test,
            SideAttdB::MIN
        );
    }
}
