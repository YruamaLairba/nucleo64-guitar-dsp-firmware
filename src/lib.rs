#![no_std]
pub mod dma;
use dma::DmaCell;

/// 2 channel AudioSample
#[derive(Copy, Clone, Debug)]
pub struct AudioSample([f32; 2]);

impl AudioSample {
    pub fn left(&self) -> f32 {
        self.0[0]
    }

    pub fn right(&self) -> f32 {
        self.0[1]
    }
    pub fn set_left(&mut self, value: f32) {
        self.0[0] = value;
    }

    pub fn set_right(&mut self, value: f32) {
        self.0[1] = value;
    }
}

impl AudioSample {
    /// Create an AudioSample from an array that represent a full i2s frame.
    pub fn from_i2s(i2s: [u16; 4]) -> Self {
        let l_msb = i2s[0];
        let l_lsb = i2s[1];
        let r_msb = i2s[2];
        let r_lsb = i2s[3];
        let l = (((l_msb as u32) << 16) + (l_lsb as u32)) as i32;
        let r = (((r_msb as u32) << 16) + (r_lsb as u32)) as i32;
        let l = (l as f32) / ((i32::MAX as f32) + 1.0);
        let r = (r as f32) / ((i32::MAX as f32) + 1.0);
        Self([l, r])
    }

    /// Create an AudioSample from an array of dma cells that contain data of a full i2s frame.
    ///
    /// The caller shoudld ensure that the array is not written elsewhere to get a value that
    /// actually correspond to i2s data.
    pub fn read_from_i2s_dma_cells(i2s: &[DmaCell<u16>; 4]) -> Self {
        let l_msb = i2s[0].read();
        let l_lsb = i2s[1].read();
        let r_msb = i2s[2].read();
        let r_lsb = i2s[3].read();
        let l = (((l_msb as u32) << 16) + (l_lsb as u32)) as i32;
        let r = (((r_msb as u32) << 16) + (r_lsb as u32)) as i32;
        let l = (l as f32) / ((i32::MAX as f32) + 1.0);
        let r = (r as f32) / ((i32::MAX as f32) + 1.0);
        Self([l, r])
    }

    pub fn into_i2s(self) -> [u16; 4] {
        let l = self.0[0];
        let r = self.0[1];
        // Note, big float are saturated by the language to fit in int
        let l = (l * ((i32::MAX as f32) + 1.0)) as i32;
        let r = (r * ((i32::MAX as f32) + 1.0)) as i32;
        let l_msb = (l >> 16) as u16;
        let l_lsb = (l & 0x0000_FFFF) as u16;
        let r_msb = (r >> 16) as u16;
        let r_lsb = (r & 0x0000_FFFF) as u16;
        [l_msb, l_lsb, r_msb, r_lsb]
    }

    /// Convert an AudioSample to raw i2s data and write it to the passed array of dma cells.
    ///
    /// The caller shoudld ensure that the array is not written elsewhere to produce i2s data that
    /// actually correspond to the AudioSample.
    pub fn write_to_i2s_dma_cells(&self, i2s: &[DmaCell<u16>; 4]) {
        let l = self.0[0];
        let r = self.0[1];
        // Note, big float are saturated by the language to fit in int
        let l = (l * ((i32::MAX as f32) + 1.0)) as i32;
        let r = (r * ((i32::MAX as f32) + 1.0)) as i32;
        let l_msb = (l >> 16) as u16;
        let l_lsb = (l & 0x0000_FFFF) as u16;
        let r_msb = (r >> 16) as u16;
        let r_lsb = (r & 0x0000_FFFF) as u16;
        i2s[0].write(l_msb);
        i2s[1].write(l_lsb);
        i2s[2].write(r_msb);
        i2s[3].write(r_lsb);
    }
}

impl From<[f32; 2]> for AudioSample {
    fn from(value: [f32; 2]) -> Self {
        Self(value)
    }
}

impl From<AudioSample> for [f32; 2] {
    fn from(value: AudioSample) -> Self {
        value.0
    }
}
