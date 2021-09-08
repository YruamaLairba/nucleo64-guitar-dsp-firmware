use core::fmt;
///Error returned when trying to scale a value into InVoldB.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum InVoldBScaleError {
    ///The given range is null.
    NullRange,
    ///The given value is outside the given range.
    OutOfRange,
}

/// Abstraction to represent input volume in dB.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct InVoldB {
    inner: u8,
}
impl InVoldB {
    ///Instanciate an `InVoldB` from it's underlaying representation.
    ///
    ///# Safety
    ///The raw value must be in between `0b00000` and `0b11111`.
    pub const unsafe fn from_raw_unchecked(raw: u8) -> Self {
        Self { inner: raw }
    }
    ///Instanciate an `InVoldB` from it's underlaying representation.
    ///
    ///The raw value is saturated to fit in the range.
    pub const fn from_raw(raw: u8) -> Self {
        if raw > InVoldB::MAX.inner {
            InVoldB::MAX
        } else {
            Self { inner: raw }
        }
    }
    ///Return the raw underlaying representation
    pub const fn into_raw(self) -> u8 {
        self.inner
    }
    /// Scale a value into a InVoldB. This function output an error when the input range is null or
    /// when the input is outside the range
    pub fn from_scaled(
        low_limit: i16,
        high_limit: i16,
        input: i16,
    ) -> Result<Self, InVoldBScaleError> {
        if low_limit == high_limit {
            return Err(InVoldBScaleError::NullRange);
        } else if (low_limit < high_limit && (input < low_limit || input > high_limit))
            || (low_limit > high_limit && (input > low_limit || input < high_limit))
        {
            return Err(InVoldBScaleError::OutOfRange);
        }

        const MIN: u8 = InVoldB::MIN.inner;
        const MAX: u8 = InVoldB::MAX.inner;
        let r1 = high_limit - low_limit;
        let r2 = MAX - MIN;
        let res = (((input - low_limit) * r2 as i16) + r1 / 2) / (r1) + MIN as i16;
        let res = unsafe { InVoldB::from_raw_unchecked(res as _) };
        Ok(res)
    }

    ///Increase the value by one step. Saturated to `InVoldB::MAX`.
    pub fn increase(&mut self) {
        if self.inner < InVoldB::MAX.inner {
            self.inner += 1;
        }
    }

    ///Decrease the value by one step. Saturated to `InVoldB::MIN`.
    pub fn decrease(&mut self) {
        if self.inner > InVoldB::MIN.inner {
            self.inner -= 1;
        }
    }

    ///Represent a value of -34.5dB
    pub const N34DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00000) };
    ///Represent a value of -33dB
    pub const N33DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00001) };
    ///Represent a value of -31.5dB
    pub const N31DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00010) };
    ///Represent a value of -30dB
    pub const N30DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00011) };
    ///Represent a value of -28.5dB
    pub const N28DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00100) };
    ///Represent a value of -27dB
    pub const N27DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00101) };
    ///Represent a value of -25.5dB
    pub const N25DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00110) };
    ///Represent a value of -24dB
    pub const N24DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b00111) };
    ///Represent a value of -22.5dB
    pub const N22DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01000) };
    ///Represent a value of -21dB
    pub const N21DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01001) };
    ///Represent a value of -19.5dB
    pub const N19DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01010) };
    ///Represent a value of -18dB
    pub const N18DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01011) };
    ///Represent a value of -16.5dB
    pub const N16DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01100) };
    ///Represent a value of -15dB
    pub const N15DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01101) };
    ///Represent a value of -13.5dB
    pub const N13DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01110) };
    ///Represent a value of -12dB
    pub const N12DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b01111) };
    ///Represent a value of -10.5dB
    pub const N10DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10000) };
    ///Represent a value of -9dB
    pub const N9DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10001) };
    ///Represent a value of -7.5dB
    pub const N7DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10010) };
    ///Represent a value of -6dB
    pub const N6DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10011) };
    ///Represent a value of -4.5dB
    pub const N4DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10100) };
    ///Represent a value of -3dB
    pub const N3DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10101) };
    ///Represent a value of -1.5dB
    pub const N1DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10110) };
    ///Represent a value of +0dB
    pub const P0DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b10111) };
    ///Represent a value of +1.5dB
    pub const P1DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11000) };
    ///Represent a value of +3dB
    pub const P3DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11001) };
    ///Represent a value of +4.5dB
    pub const P4DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11010) };
    ///Represent a value of +6dB
    pub const P6DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11011) };
    ///Represent a value of +7.5dB
    pub const P7DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11100) };
    ///Represent a value of +9dB
    pub const P9DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11101) };
    ///Represent a value of +10.5dB
    pub const P10DB5: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11110) };
    ///Represent a value of +12dB
    pub const P12DB: InVoldB = unsafe { InVoldB::from_raw_unchecked(0b11111) };
    ///Represent the smallest value
    pub const MIN: InVoldB = InVoldB::N34DB5;
    ///Represent the greatest value
    pub const MAX: InVoldB = InVoldB::P12DB;
}
impl fmt::Display for InVoldB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let fmt = match self.inner {
            0b00000 => "-34.5",
            0b00001 => "-33",
            0b00010 => "-31.5",
            0b00011 => "-30",
            0b00100 => "-28.5",
            0b00101 => "-27",
            0b00110 => "-25.5",
            0b00111 => "-24",
            0b01000 => "-22.5",
            0b01001 => "-21",
            0b01010 => "-19.5",
            0b01011 => "-18",
            0b01100 => "-16.5",
            0b01101 => "-15",
            0b01110 => "-13.5",
            0b01111 => "-12",
            0b10000 => "-10.5",
            0b10001 => "-9",
            0b10010 => "-7.5",
            0b10011 => "-6",
            0b10100 => "-4.5",
            0b10101 => "-3",
            0b10110 => "-1.5",
            0b10111 => "+0",
            0b11000 => "+1.5",
            0b11001 => "+3",
            0b11010 => "+4.5",
            0b11011 => "+6",
            0b11100 => "+7.5",
            0b11101 => "+9",
            0b11110 => "+10.5",
            0b11111 => "+12",
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
        let db = InVoldB::from_scaled(0, 255, 0).unwrap().inner;
        let expected = InVoldB::MIN.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = InVoldB::from_scaled(0, 255, 255).unwrap().inner;
        let expected = InVoldB::MAX.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = InVoldB::from_scaled(255, 0, 0).unwrap().inner;
        let expected = InVoldB::MAX.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = InVoldB::from_scaled(255, 0, 255).unwrap().inner;
        let expected = InVoldB::MIN.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = InVoldB::from_scaled(128, 128, 128).unwrap_err();
        let expected = InVoldBScaleError::NullRange;
        assert!(db == expected, "Got {:?},expected {:?}", db, expected);
        let db = InVoldB::from_scaled(0, 127, 128).unwrap_err();
        let expected = InVoldBScaleError::OutOfRange;
        assert!(db == expected, "Got {:?},expected {:?}", db, expected);
    }
    #[test]
    fn increase_decrease_saturation_test() {
        let mut test = InVoldB::MAX;
        test.increase();
        assert!(
            test == InVoldB::MAX,
            "Got {}, expected {}",
            test,
            InVoldB::MAX
        );
        let mut test = InVoldB::MIN;
        test.decrease();
        assert!(
            test == InVoldB::MIN,
            "Got {}, expected {}",
            test,
            InVoldB::MIN
        );
    }
}
