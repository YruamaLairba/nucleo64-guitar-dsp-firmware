///This macro is the template for raw bits write to a field
macro_rules! impl_bits {
    (unsafe, $ret:ty, $lenght:literal, $shift:literal) => {
        /// # Safety
        ///
        /// Some bit combinations are invalid, please read the datasheet.
        #[must_use]
        pub unsafe fn bits(mut self, value: u8) -> $ret {
            let mask = !((!0) << $lenght) << $shift;
            self.cmd.data = self.cmd.data & !mask | (value as u16) << $shift & mask;
            self.cmd
        }
    };
    ($ret:ty, $lenght:literal, $shift:literal) => {
        #[must_use]
        pub fn bits(mut self, value: u8) -> $ret {
            let mask = !((!0) << $lenght) << $shift;
            self.cmd.data = self.cmd.data & !mask | (value as u16) << $shift & mask;
            self.cmd
        }
    };
}

///Template for raw bit write to one bit field
macro_rules! impl_bit {
    ($ret:ty, $pos:literal) => {
        ///Writes raw bit to the field.
        #[must_use]
        pub fn bit(mut self, value: bool) -> $ret {
            self.cmd.data = self.cmd.data & !(1 << $pos) | (value as u16) << $pos;
            self.cmd
        }
    };
}

///Template for setting a bit into a one bit field
macro_rules! impl_set_bit {
    ($alias:ident, $ret:ty, $pos:literal) => {
        #[must_use]
        pub fn $alias(mut self) -> $ret {
            self.cmd.data |= 0b1 << $pos;
            self.cmd
        }
    };
    ($ret:ty, $pos:literal) => {
        /// Sets the bit field.
        #[must_use]
        pub fn set_bit(mut self) -> $ret {
            self.cmd.data |= 0b1 << $pos;
            self.cmd
        }
    };
}

///Template for enabling a bit into a one bit field
macro_rules! impl_enable {
    ($ret:ty, $pos:literal) => {
        #[must_use]
        pub fn enable(mut self) -> $ret {
            self.cmd.data |= 0b1 << $pos;
            self.cmd
        }
    };
}

///Template for clearing a bit into a one bit field
macro_rules! impl_clear_bit {
    ($alias:ident, $ret:ty, $pos:literal) => {
        #[must_use]
        pub fn $alias(mut self) -> $ret {
            self.cmd.data &= !(0b1 << $pos);
            self.cmd
        }
    };
    ($ret:ty, $pos:literal) => {
        /// Clears the bit field.
        #[must_use]
        pub fn clear_bit(mut self) -> $ret {
            self.cmd.data &= !(0b1 << $pos);
            self.cmd
        }
    };
}

///Template for disabling a bit into a one bit field
macro_rules! impl_disable {
    ($ret:ty, $pos:literal) => {
        #[must_use]
        pub fn disable(mut self) -> $ret {
            self.cmd.data &= !(0b1 << $pos);
            self.cmd
        }
    };
}

macro_rules! impl_bitsetters {
    ($ret:ty, $pos:literal) => {
        impl_bit!($ret, $pos);
        impl_set_bit!($ret, $pos);
        impl_clear_bit!($ret, $pos);
        impl_enable!($ret, $pos);
        impl_disable!($ret, $pos);
    };
}

macro_rules! impl_toggle_writer {
    ($name:ident$(<$mark:tt>)?, $ret:ty, $pos:literal) => {
        ///Field writer.
        pub struct $name $(<$mark>)? {
            cmd: $ret,
        }

        impl$(<$mark>)? $name$(<$mark>)? {
            impl_bitsetters!($ret, $pos);
        }
    };
}

#[cfg(test)]
mod tests {
    #[derive(Debug, Eq, PartialEq, Clone, Copy)]
    pub struct Cmd {
        data: u16,
    }

    impl Cmd {
        fn bits_w(self) -> BitsW {
            BitsW { cmd: self }
        }
        fn bit_w(self) -> BitW {
            BitW { cmd: self }
        }
    }

    pub struct BitsW {
        cmd: Cmd,
    }

    impl BitsW {
        impl_bits!(Cmd, 5, 2);
    }

    impl_toggle_writer!(BitW, Cmd, 1);

    #[test]
    fn macro_tests() {
        let expect = Cmd { data: 0b111_1100 };
        let test = Cmd { data: 0 }.bits_w().bits(0b111_11);
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
        let expect = Cmd { data: 0b111_1100 };
        let test = Cmd { data: 0 }.bits_w().bits(!0);
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
        let expect = Cmd { data: 0b10 };
        let test = Cmd { data: 0 }.bit_w().bit(true);
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
        let expect = Cmd { data: 0b10 };
        let test = Cmd { data: 0 }.bit_w().set_bit();
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
        let expect = Cmd { data: 0b10 };
        let test = Cmd { data: 0 }.bit_w().enable();
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
        let expect = Cmd { data: 0b101 };
        let test = Cmd { data: 0b111 }.bit_w().clear_bit();
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
        let expect = Cmd { data: 0b101 };
        let test = Cmd { data: 0b111 }.bit_w().disable();
        assert_eq!(
            test, expect,
            "Got {:#b}, expected {:#b}",
            test.data, expect.data
        );
    }
}
