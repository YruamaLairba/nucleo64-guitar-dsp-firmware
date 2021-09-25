#![no_std]
use core::mem::size_of;
use stm32f4xx_hal::stm32::{I2S2EXT, SPI2};
use wm8731_alt::interface::WriteFrame;
use wm8731_alt::prelude::*;
use wm8731_alt::Wm8731;

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
            l: smpl.l.rotate_left(16) as i32,
            r: smpl.r.rotate_left(16) as i32,
        }
    }
}

///Stereo sample representation for i2s transfert
#[derive(Copy, Clone)]
//for DMA use i need a guaranted layout
#[repr(C, align(4))]
pub struct I2sSample {
    pub l: u32,
    pub r: u32,
}

impl I2sSample {
    pub const fn new() -> Self {
        Self { l: 0, r: 0 }
    }
}

impl From<StereoSample> for I2sSample {
    fn from(smpl: StereoSample) -> Self {
        Self {
            l: smpl.l.rotate_right(16) as u32,
            r: smpl.r.rotate_right(16) as u32,
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

pub fn setup_spi2(spi2: &mut SPI2, i2sdiv: u8, odd: bool, mck: bool) {
    //i2s2 interrupt
    spi2.cr2.modify(|_, w| {
        w.txeie().clear_bit();
        w.rxneie().set_bit();
        w.errie().set_bit()
    });
    //setup spi2 peripheral into i2s mode
    spi2.i2spr.modify(|_, w| {
        unsafe { w.i2sdiv().bits(i2sdiv) };
        w.odd().bit(odd);
        w.mckoe().bit(mck)
    });
    spi2.i2scfgr.modify(|_, w| {
        w.i2smod().i2smode(); //
        w.i2scfg().master_rx(); //
        w.pcmsync().long(); //
        w.i2sstd().philips(); //
        w.ckpol().idle_high(); //
        w.datlen().twenty_four_bit(); //
        w.chlen().thirty_two_bit(); //
        w.i2se().disabled()
    });
}

pub fn setup_i2s2ext(i2s2ext: &mut I2S2EXT, i2sdiv: u8, odd: bool, mck: bool) {
    //i2s2_ext interrupt
    i2s2ext.cr2.modify(|_, w| {
        w.txeie().set_bit();
        w.rxneie().clear_bit();
        w.errie().set_bit()
    });
    //setup i2s2ext peripheral
    i2s2ext.i2spr.modify(|_, w| {
        unsafe { w.i2sdiv().bits(i2sdiv) };
        w.odd().bit(odd);
        w.mckoe().bit(mck)
    });
    i2s2ext.i2scfgr.modify(|_, w| {
        w.i2smod().i2smode(); //
        w.i2scfg().slave_tx(); //
        w.pcmsync().long(); //
        w.i2sstd().philips(); //
        w.ckpol().idle_high(); //
        w.datlen().twenty_four_bit(); //
        w.chlen().thirty_two_bit(); //
        w.i2se().disabled()
    });
}

pub fn setup_wm8731<T>(wm8731: &mut Wm8731<T>)
where
    T: WriteFrame,
{
    //line input: unmute, apply setup for both channel, set volume to 0dB.
    let lli = left_line_in()
        .inmute()
        .clear_bit()
        .inboth()
        .set_bit()
        .invol()
        .db(InVoldB::P0DB)
        .into_command();
    //rprintln!("{:016b}", u16::from(Frame::from(lli)));
    wm8731.send(lli);
    //headphone out: volume -12 dB, no zero cross detection, apply setup to both channel
    let lho = left_headphone_out()
        .hpvol()
        .db(HpVoldB::N12DB)
        .zcen()
        .clear_bit()
        .hpboth()
        .set_bit()
        .into_command();
    //rprintln!("{:016b}", u16::from(Frame::from(lho)));
    wm8731.send(lho);
    //analogue audio path control
    let mut aap = analogue_audio_path();
    aap = aap.micboost().disable();
    aap = aap.mutemic().enable();
    aap = aap.insel().line();
    aap = aap.bypass().disable();
    aap = aap.dacsel().select();
    aap = aap.sidetone().disable();
    let aap = aap.into_command();
    //rprintln!("{:016b}", u16::from(Frame::from(aap)));
    wm8731.send(aap);
    //digital audio path control default adchp hpor ans deemp are ok
    let mut dap = digital_audio_path();
    dap = dap.dacmu().disable();
    let dap = dap.into_command();
    //rprintln!("{:016b}", u16::from(Frame::from(dap)));
    wm8731.send(dap);
    //power down: by default all is power down, ie all field are set to 1
    let mut pd = power_down();
    pd = pd.lineinpd().clear_bit();
    pd = pd.adcpd().clear_bit();
    pd = pd.dacpd().clear_bit();
    pd = pd.outpd().set_bit();
    pd = pd.oscpd().set_bit();
    pd = pd.clkoutpd().set_bit();
    pd = pd.poweroff().clear_bit();
    //rprintln!("{:016b}", u16::from(Frame::from(pd)));
    wm8731.send(pd.into_command());
    //digital audi interface
    let mut dai = digital_audio_interface();
    dai = dai.format().i2s();
    dai = dai.iwl().iwl_32_bits();
    dai = dai.lrp().clear_bit();
    dai = dai.lrswap().disable();
    dai = dai.ms().slave();
    dai = dai.bclkinv().disable();
    let dai = dai.into_command();
    //rprintln!("{:016b}", u16::from(Frame::from(dai)));
    wm8731.send(dai);

    //Sampling control: use the raw method
    let s = sampling();
    let s = s.usb_normal().normal();
    let s = s.bosr().clear_bit(); //for 256 fs
    let s = s.sr().sr_0b0000();
    let s = s.into_command();
    //rprintln!("{:016b}", u16::from(Frame::from(s)));
    wm8731.send(s);
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
