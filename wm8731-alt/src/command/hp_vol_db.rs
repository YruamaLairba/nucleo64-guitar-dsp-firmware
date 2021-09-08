use core::fmt;

///Error returned when trying to scale a value into HpVoldB.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum HpVoldBScaleError {
    ///The given range is null.
    NullRange,
    ///The given value is outside the given range.
    OutOfRange,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
/// Headphone volume in dB
pub struct HpVoldB {
    inner: u8,
}
impl HpVoldB {
    ///Instanciate an `HpVoldB` from it's underlaying representation.
    ///
    ///# Safety
    ///The raw value must be in between `0b0101111` and `0b1111111`.
    pub const unsafe fn from_raw_unchecked(raw: u8) -> Self {
        Self { inner: raw }
    }
    ///Instanciate an `HpVoldB` from it's underlaying representation.
    ///
    ///The raw value is clamped to fit in the range.
    pub const fn from_raw(raw: u8) -> Self {
        let raw = if raw < HpVoldB::MIN.inner {
            HpVoldB::MIN.inner
        } else if raw > HpVoldB::MAX.inner {
            HpVoldB::MAX.inner
        } else {
            raw
        };
        Self { inner: raw }
    }
    ///Return the raw underlaying representation
    pub const fn into_raw(self) -> u8 {
        self.inner
    }
    /// Scale a value into a HpVoldB. This function output an error when the input range is null or
    /// when the input is outside the range
    pub fn from_scaled(
        low_limit: i16,
        high_limit: i16,
        input: i16,
    ) -> Result<Self, HpVoldBScaleError> {
        if low_limit == high_limit {
            return Err(HpVoldBScaleError::NullRange);
        } else if (low_limit < high_limit && (input < low_limit || input > high_limit))
            || (low_limit > high_limit && (input > low_limit || input < high_limit))
        {
            return Err(HpVoldBScaleError::OutOfRange);
        }

        const MIN: u8 = HpVoldB::MIN.inner;
        const MAX: u8 = HpVoldB::MAX.inner;
        let r1 = high_limit - low_limit;
        let r2 = MAX - MIN;
        let res = (((input - low_limit) * r2 as i16) + r1 / 2) / (r1) + MIN as i16;
        let res = unsafe { HpVoldB::from_raw_unchecked(res as _) };
        Ok(res)
    }

    ///Increase the value by one step. Saturated to `HpVoldB::MAX`.
    pub fn increase(&mut self) {
        if self.inner < HpVoldB::MAX.inner {
            self.inner += 1;
        }
    }

    ///Decrease the value by one step. Saturated to `HpVoldB::MIN`.
    pub fn decrease(&mut self) {
        if self.inner > HpVoldB::MIN.inner {
            self.inner -= 1;
        }
    }
    ///Represent a mute value
    pub const MUTE: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0101111) };
    ///Represent a value of -73dB
    pub const N73DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110000) };
    ///Represent a value of -72dB
    pub const N72DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110001) };
    ///Represent a value of -71dB
    pub const N71DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110010) };
    ///Represent a value of -70dB
    pub const N70DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110011) };
    ///Represent a value of -69dB
    pub const N69DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110100) };
    ///Represent a value of -68dB
    pub const N68DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110101) };
    ///Represent a value of -67dB
    pub const N67DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110110) };
    ///Represent a value of -66dB
    pub const N66DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0110111) };
    ///Represent a value of -65dB
    pub const N65DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111000) };
    ///Represent a value of -64dB
    pub const N64DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111001) };
    ///Represent a value of -63dB
    pub const N63DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111010) };
    ///Represent a value of -62dB
    pub const N62DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111011) };
    ///Represent a value of -61dB
    pub const N61DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111100) };
    ///Represent a value of -60dB
    pub const N60DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111101) };
    ///Represent a value of -59dB
    pub const N59DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111110) };
    ///Represent a value of -58dB
    pub const N58DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b0111111) };
    ///Represent a value of -57dB
    pub const N57DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000000) };
    ///Represent a value of -56dB
    pub const N56DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000001) };
    ///Represent a value of -55dB
    pub const N55DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000010) };
    ///Represent a value of -54dB
    pub const N54DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000011) };
    ///Represent a value of -53dB
    pub const N53DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000100) };
    ///Represent a value of -52dB
    pub const N52DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000101) };
    ///Represent a value of -51dB
    pub const N51DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000110) };
    ///Represent a value of -50dB
    pub const N50DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1000111) };
    ///Represent a value of -49dB
    pub const N49DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001000) };
    ///Represent a value of -48dB
    pub const N48DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001001) };
    ///Represent a value of -47dB
    pub const N47DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001010) };
    ///Represent a value of -46dB
    pub const N46DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001011) };
    ///Represent a value of -45dB
    pub const N45DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001100) };
    ///Represent a value of -44dB
    pub const N44DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001101) };
    ///Represent a value of -43dB
    pub const N43DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001110) };
    ///Represent a value of -42dB
    pub const N42DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1001111) };
    ///Represent a value of -41dB
    pub const N41DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010000) };
    ///Represent a value of -40dB
    pub const N40DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010001) };
    ///Represent a value of -39dB
    pub const N39DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010010) };
    ///Represent a value of -38dB
    pub const N38DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010011) };
    ///Represent a value of -37dB
    pub const N37DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010100) };
    ///Represent a value of -36dB
    pub const N36DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010101) };
    ///Represent a value of -35dB
    pub const N35DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010110) };
    ///Represent a value of -34dB
    pub const N34DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1010111) };
    ///Represent a value of -33dB
    pub const N33DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011000) };
    ///Represent a value of -32dB
    pub const N32DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011001) };
    ///Represent a value of -31dB
    pub const N31DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011010) };
    ///Represent a value of -30dB
    pub const N30DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011011) };
    ///Represent a value of -29dB
    pub const N29DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011100) };
    ///Represent a value of -28dB
    pub const N28DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011101) };
    ///Represent a value of -27dB
    pub const N27DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011110) };
    ///Represent a value of -26dB
    pub const N26DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1011111) };
    ///Represent a value of -25dB
    pub const N25DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100000) };
    ///Represent a value of -24dB
    pub const N24DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100001) };
    ///Represent a value of -23dB
    pub const N23DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100010) };
    ///Represent a value of -22dB
    pub const N22DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100011) };
    ///Represent a value of -21dB
    pub const N21DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100100) };
    ///Represent a value of -20dB
    pub const N20DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100101) };
    ///Represent a value of -19dB
    pub const N19DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100110) };
    ///Represent a value of -18dB
    pub const N18DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1100111) };
    ///Represent a value of -17dB
    pub const N17DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101000) };
    ///Represent a value of -16dB
    pub const N16DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101001) };
    ///Represent a value of -15dB
    pub const N15DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101010) };
    ///Represent a value of -14dB
    pub const N14DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101011) };
    ///Represent a value of -13dB
    pub const N13DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101100) };
    ///Represent a value of -12dB
    pub const N12DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101101) };
    ///Represent a value of -11dB
    pub const N11DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101110) };
    ///Represent a value of -10dB
    pub const N10DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1101111) };
    ///Represent a value of -9dB
    pub const N9DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110000) };
    ///Represent a value of -8dB
    pub const N8DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110001) };
    ///Represent a value of -7dB
    pub const N7DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110010) };
    ///Represent a value of -6dB
    pub const N6DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110011) };
    ///Represent a value of -5dB
    pub const N5DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110100) };
    ///Represent a value of -4dB
    pub const N4DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110101) };
    ///Represent a value of -3dB
    pub const N3DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110110) };
    ///Represent a value of -2dB
    pub const N2DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1110111) };
    ///Represent a value of -1dB
    pub const N1DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111000) };
    ///Represent a value of +0dB
    pub const P0DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111001) };
    ///Represent a value of +1dB
    pub const P1DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111010) };
    ///Represent a value of +2dB
    pub const P2DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111011) };
    ///Represent a value of +3dB
    pub const P3DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111100) };
    ///Represent a value of +4dB
    pub const P4DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111101) };
    ///Represent a value of +5dB
    pub const P5DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111110) };
    ///Represent a value of +6dB
    pub const P6DB: HpVoldB = unsafe { HpVoldB::from_raw_unchecked(0b1111111) };
    ///Represent the smallest value
    pub const MIN: HpVoldB = HpVoldB::MUTE;
    ///Represent the greatest value
    pub const MAX: HpVoldB = HpVoldB::P6DB;
}
impl fmt::Display for HpVoldB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let fmt = match self.inner {
            0b0110000 => "-73",
            0b0110001 => "-72",
            0b0110010 => "-71",
            0b0110011 => "-70",
            0b0110100 => "-69",
            0b0110101 => "-68",
            0b0110110 => "-67",
            0b0110111 => "-66",
            0b0111000 => "-65",
            0b0111001 => "-64",
            0b0111010 => "-63",
            0b0111011 => "-62",
            0b0111100 => "-61",
            0b0111101 => "-60",
            0b0111110 => "-59",
            0b0111111 => "-58",
            0b1000000 => "-57",
            0b1000001 => "-56",
            0b1000010 => "-55",
            0b1000011 => "-54",
            0b1000100 => "-53",
            0b1000101 => "-52",
            0b1000110 => "-51",
            0b1000111 => "-50",
            0b1001000 => "-49",
            0b1001001 => "-48",
            0b1001010 => "-47",
            0b1001011 => "-46",
            0b1001100 => "-45",
            0b1001101 => "-44",
            0b1001110 => "-43",
            0b1001111 => "-42",
            0b1010000 => "-41",
            0b1010001 => "-40",
            0b1010010 => "-39",
            0b1010011 => "-38",
            0b1010100 => "-37",
            0b1010101 => "-36",
            0b1010110 => "-35",
            0b1010111 => "-34",
            0b1011000 => "-33",
            0b1011001 => "-32",
            0b1011010 => "-31",
            0b1011011 => "-30",
            0b1011100 => "-29",
            0b1011101 => "-28",
            0b1011110 => "-27",
            0b1011111 => "-26",
            0b1100000 => "-25",
            0b1100001 => "-24",
            0b1100010 => "-23",
            0b1100011 => "-22",
            0b1100100 => "-21",
            0b1100101 => "-20",
            0b1100110 => "-19",
            0b1100111 => "-18",
            0b1101000 => "-17",
            0b1101001 => "-16",
            0b1101010 => "-15",
            0b1101011 => "-14",
            0b1101100 => "-13",
            0b1101101 => "-12",
            0b1101110 => "-11",
            0b1101111 => "-10",
            0b1110000 => "-9",
            0b1110001 => "-8",
            0b1110010 => "-7",
            0b1110011 => "-6",
            0b1110100 => "-5",
            0b1110101 => "-4",
            0b1110110 => "-3",
            0b1110111 => "-2",
            0b1111000 => "-1",
            0b1111001 => "+0",
            0b1111010 => "+1",
            0b1111011 => "+2",
            0b1111100 => "+3",
            0b1111101 => "+4",
            0b1111110 => "+5",
            0b1111111 => "+6",
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
        let db = HpVoldB::from_scaled(0, 255, 0).unwrap().inner;
        let expected = HpVoldB::MIN.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = HpVoldB::from_scaled(0, 255, 255).unwrap().inner;
        let expected = HpVoldB::MAX.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = HpVoldB::from_scaled(255, 0, 0).unwrap().inner;
        let expected = HpVoldB::MAX.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = HpVoldB::from_scaled(255, 0, 255).unwrap().inner;
        let expected = HpVoldB::MIN.inner;
        assert!(db == expected, "Got {:#b},expected {:#b}", db, expected);
        let db = HpVoldB::from_scaled(128, 128, 128).unwrap_err();
        let expected = HpVoldBScaleError::NullRange;
        assert!(db == expected, "Got {:?},expected {:?}", db, expected);
        let db = HpVoldB::from_scaled(0, 127, 128).unwrap_err();
        let expected = HpVoldBScaleError::OutOfRange;
        assert!(db == expected, "Got {:?},expected {:?}", db, expected);
    }
    #[test]
    fn increase_decrease_saturation_test() {
        let mut test = HpVoldB::MAX;
        test.increase();
        assert!(
            test == HpVoldB::MAX,
            "Got {}, expected {}",
            test,
            HpVoldB::MAX
        );
        let mut test = HpVoldB::MIN;
        test.decrease();
        assert!(
            test == HpVoldB::MIN,
            "Got {}, expected {}",
            test,
            HpVoldB::MIN
        );
    }
}
