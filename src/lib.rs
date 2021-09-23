#![no_std]
use core::mem::size_of;

///Stereo sample representation for DSP calculation.
#[derive(Copy, Clone)]
pub struct StereoSample {
    pub l: i32,
    pub r: i32,
}

impl StereoSample {
    pub const fn new() -> Self {
        Self { l: 0, r: 0 }
    }
}

impl From<I2sSample> for StereoSample {
    fn from(smpl: I2sSample) -> Self {
        Self {
            l: smpl.l.rotate_left(16),
            r: smpl.r.rotate_left(16),
        }
    }
}

///Stereo sample representation for i2s transfert
#[derive(Copy, Clone)]
//for DMA use i need a guaranted layout
#[repr(C,align(4))]
pub struct I2sSample {
    l: i32,
    r: i32,
}

impl I2sSample {
    pub const fn new() -> Self {
        Self { l: 0, r: 0 }
    }
}

impl From<StereoSample> for I2sSample {
    fn from(smpl: StereoSample) -> Self {
        Self {

            l: smpl.l.rotate_right(16),
            r: smpl.r.rotate_right(16),
        }
    }
}

const BUF_SIZE: usize = 128;
/// Buffer for DMA
#[repr(C, align(4))]
pub union Buffer {
    i2s_smpl: [I2sSample; BUF_SIZE],
    au16: [u16; BUF_SIZE * (size_of::<I2sSample>() / 2)],
}

impl Buffer {
    pub const fn new() -> Self {
        Self {
            i2s_smpl: [I2sSample::new(); BUF_SIZE],
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
