//! examples/init.rs

//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use core::panic::PanicInfo;
use nucleo64_guitar_dsp_firmware::*;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::gpio::gpioa::PA5;
use stm32f4xx_hal::gpio::gpioa::PA7;
use stm32f4xx_hal::gpio::gpiob::PB2;
use stm32f4xx_hal::gpio::Alternate;
use stm32f4xx_hal::gpio::Output;
use stm32f4xx_hal::gpio::PushPull;
use stm32f4xx_hal::gpio::AF5;
use stm32f4xx_hal::spi::NoMiso;
use stm32f4xx_hal::spi::Spi;
use stm32f4xx_hal::stm32::spi1::sr::CHSIDE_A;
use stm32f4xx_hal::stm32::{EXTI, I2S2EXT, SPI1, SPI2};
use stm32f4xx_hal::{prelude::*, spi, stm32};
use wm8731_alt::interface::SPIInterfaceU8;
use wm8731_alt::prelude::*;
use wm8731_alt::Wm8731;
//use stm32f4::stm32f411;

//PLLI2S clock configuration
const PLLI2SM: u8 = 5;
const PLLI2SN: u16 = 192;
const PLLI2SR: u8 = 5;

//Clock configuration of the used i2s interface
const I2SDIV: u8 = 2;
const ODD: bool = true;

//generate Master Clock ? Modifying this require to adapt the i2s clock
const MCK: bool = true;

type MyWm8731 = Wm8731<
    SPIInterfaceU8<
        Spi<SPI1, (PA5<Alternate<AF5>>, NoMiso, PA7<Alternate<AF5>>)>,
        PB2<Output<PushPull>>,
    >,
>;

const BUF_SIZE: usize = 4;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        spi2: SPI2,
        i2s2ext: I2S2EXT,
        exti: EXTI,
        wm8731: MyWm8731,
        buf: [I2sSample; BUF_SIZE],
    }
    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut buf = [I2sSample::new(); BUF_SIZE];
        rtt_init_print!();
        rprintln!("init");
        let device = cx.device;
        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let rcc = device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(96.mhz())
            .hclk(96.mhz())
            .pclk1(50.mhz())
            .pclk2(100.mhz())
            .freeze();

        //setup  and startup PLLI2S clock
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            //enable system clock for APB1 bus and SPI2 (I2S2)
            rcc.apb1enr
                .modify(|_, w| w.pwren().set_bit().spi2en().set_bit());
            //setup
            rcc.plli2scfgr.modify(|_, w| {
                w.plli2sr().bits(PLLI2SR);
                w.plli2sn().bits(PLLI2SN);
                w.plli2sm().bits(PLLI2SM)
            });
            //run the clock, doesn't work if done before freezing clock
            rcc.cr.modify(|_, w| w.plli2son().set_bit());
            //wait a stable clock
            while rcc.cr.read().plli2srdy().bit_is_clear() {}
        }

        //Setup Spi1
        let pa5 = gpioa.pa5.into_alternate_af5(); //CK
        let pa7 = gpioa.pa7.into_alternate_af5(); //MOSI
        let mut pb2 = gpiob.pb2.into_push_pull_output(); //CS
        let _ = pb2.set_high();

        let spi1_mode = spi::Mode {
            polarity: spi::Polarity::IdleHigh,
            phase: spi::Phase::CaptureOnSecondTransition, //With IdleHigh, capture on rising edge
        };

        let spi1 = Spi::spi1(
            device.SPI1,
            (pa5, spi::NoMiso, pa7),
            spi1_mode,
            1.mhz().into(),
            clocks,
        );
        let mut wm8731 = Wm8731::new(SPIInterfaceU8::new(spi1, pb2));
        setup_wm8731(&mut wm8731);

        //Setup i2s2 and i2s2_ext
        //gpio
        let _pb13 = gpiob.pb13.into_alternate_af5(); //CK
        let _pb15 = gpiob.pb15.into_alternate_af5(); //SD
        let _pb14 = gpiob.pb14.into_alternate_af6(); //ext_SD
        let _pb12 = gpiob.pb12.into_alternate_af5(); //WS
        let _pc6 = gpioc.pc6.into_alternate_af5(); //MCK

        //Setup an interrupt that can be triggered by pb12 pin
        //Note: The hal doesn't allow to manipulate interrupt for pin in aternate mode
        let syscfg = device.SYSCFG;
        let exti = device.EXTI;
        //i on pb12
        syscfg
            .exticr4
            .modify(|_, w| unsafe { w.exti12().bits(0b0001) });
        //mask EXTI0 interrupt
        exti.imr.modify(|_, w| w.mr12().set_bit());
        //trigger interrupt on rising edge
        exti.rtsr.modify(|_, w| w.tr12().set_bit());

        let mut spi2 = device.SPI2;
        setup_spi2(&mut spi2, I2SDIV, ODD, MCK);

        let mut i2s2ext = device.I2S2EXT;
        setup_i2s2ext(&mut i2s2ext, I2SDIV, ODD, MCK);

        let mut dma1 = device.DMA1;
        setup_dma1(&mut dma1, &mut buf, &mut spi2, &mut i2s2ext);

        //Active Control
        wm8731.send(active_control().active().into_command());
        //run i2s
        i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
        spi2.i2scfgr.modify(|_, w| w.i2se().enabled());
        let mut pd = power_down();
        pd = pd.lineinpd().clear_bit();
        pd = pd.adcpd().clear_bit();
        pd = pd.dacpd().clear_bit();
        pd = pd.poweroff().clear_bit();
        pd = pd.outpd().clear_bit();
        wm8731.send(pd.into_command());
        rprintln!("init done");
        init::LateResources {
            buf,
            exti,
            i2s2ext,
            spi2,
            wm8731,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        static mut X: u32 = 0;

        // Safe access to local `static mut` variable
        let _x: &'static mut u32 = X;

        rprintln!("idle");

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(resources = [buf])]
    fn process(cx: process::Context, range: core::ops::Range<usize>) {
        let buf = cx.resources.buf;
        for sample in &mut buf[range] {
            let mut smpl = StereoSample::from(*sample);
            let sum = smpl.l + smpl.r;
            smpl = StereoSample { l: sum, r: sum };
            *sample = smpl.into();
        }
    }

    #[task(binds = SPI2,  resources = [spi2,i2s2ext,exti,wm8731,buf],spawn = [process])]
    fn spi2(cx: spi2::Context) {
        static mut PREVIOUS_RX_SIDE: CHSIDE_A = CHSIDE_A::RIGHT;
        static mut PREVIOUS_TX_SIDE: CHSIDE_A = CHSIDE_A::RIGHT;
        static mut RX_DATA: [u16; 4] = [0; 4];
        static mut TX_DATA: [u16; 4] = [0; 4];
        static mut ADC_IDX: usize = 0;

        let spi2 = cx.resources.spi2;
        let i2s2ext = cx.resources.i2s2ext;
        let exti = cx.resources.exti;
        let buf = cx.resources.buf;
        if spi2.sr.read().fre().bit() {
            rprintln!("Frame Error");
        }
        if spi2.sr.read().ovr().bit() {
            rprintln!("Overrun");
            //this sequence reset the interrupt
            let _ = spi2.dr.read().bits();
            let _ = spi2.sr.read().bits();
        }
        if spi2.sr.read().udr().bit() {
            rprintln!("underrun");
            //clear the interrupt
            let _ = spi2.sr.read().bits();
        }
        if spi2.sr.read().rxne().bit() {
            let data = spi2.dr.read().dr().bits();
            let side = spi2.sr.read().chside().variant();
            if *PREVIOUS_RX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::LEFT {
                //left msb
                RX_DATA[0] = data;
            } else if *PREVIOUS_RX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::LEFT {
                //left lsb
                RX_DATA[1] = data;
            } else if *PREVIOUS_RX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::RIGHT {
                //right msb
                RX_DATA[2] = data;
            } else if *PREVIOUS_RX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::RIGHT {
                //right lsb
                RX_DATA[3] = data;
                let left = (RX_DATA[0] as u32) << 16 | (RX_DATA[1] as u32);
                let right = (RX_DATA[2] as u32) << 16 | (RX_DATA[3] as u32);
                buf[*ADC_IDX] = I2sSample { l: left, r: right };
                if *ADC_IDX == (BUF_SIZE - 1) {
                    let _ = cx.spawn.process(BUF_SIZE / 2..BUF_SIZE);
                } else if *ADC_IDX == (BUF_SIZE / 2 - 1) {
                    let _ = cx.spawn.process(0..BUF_SIZE / 2);
                }
                *ADC_IDX = (*ADC_IDX + 1) & (BUF_SIZE - 1);

                //let _ = cx.spawn.process(false);
                //rprintln!(
                //    "RX: {:016b} {:016b} {:016b} {:016b}",
                //    RX_DATA[0],
                //    RX_DATA[1],
                //    RX_DATA[2],
                //    RX_DATA[3]
                //);
            }

            *PREVIOUS_RX_SIDE = side;
            //rprintln!("{:b}", data);
        }

        if i2s2ext.sr.read().fre().bit() {
            rprintln!("ext Frame Error");
            //resynchronization
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().disabled());
            //couldn't find to get word select value
            let ws = unsafe {
                let gpiob = &(*stm32::GPIOB::ptr());
                gpiob.idr.read().idr12().bit()
            };
            if ws {
                i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
                rprintln!("Resynced (I2S2EXT)");
            } else {
                exti.imr.modify(|_, w| w.mr12().set_bit());
            }
        }
        if i2s2ext.sr.read().ovr().bit() {
            rprintln!("ext Overrun");
            //this sequence reset the interrupt
            let _ = i2s2ext.dr.read().bits();
            let _ = i2s2ext.sr.read().bits();
        }
        if i2s2ext.sr.read().udr().bit() {
            rprintln!("ext underrun");
            //clear the interrupt
            let _ = i2s2ext.sr.read().bits();
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().disabled());
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
        }
        if i2s2ext.sr.read().txe().bit() {
            //let _data = i2s2ext.dr.read().dr().bits();
            let side = i2s2ext.sr.read().chside().variant();
            if *PREVIOUS_TX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::LEFT {
                //left msb
                i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[0]));
            } else if *PREVIOUS_TX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::LEFT {
                //left lsb
                i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[1]));
            } else if *PREVIOUS_TX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::RIGHT {
                //right msb
                i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[2]));
            } else if *PREVIOUS_TX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::RIGHT {
                //right lsb, end of audio frame
                let dac_idx = (*ADC_IDX + 2) & (BUF_SIZE - 1);
                i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[3]));
                TX_DATA[0] = (buf[dac_idx].l >> 16) as u16;
                TX_DATA[1] = buf[dac_idx].l as u16;
                TX_DATA[2] = (buf[dac_idx].r >> 16) as u16;
                TX_DATA[3] = buf[dac_idx].r as u16;
            } else {
                unreachable!()
            };

            *PREVIOUS_TX_SIDE = side;
        }
    }
    #[task(binds = EXTI15_10,resources = [i2s2ext,exti])]
    fn exti15_10(cx: exti15_10::Context) {
        let i2s2ext = cx.resources.i2s2ext;
        let exti = cx.resources.exti;
        let ws = unsafe {
            let gpiob = &(*stm32::GPIOB::ptr());
            gpiob.idr.read().idr12().bit()
        };
        //erase the event
        exti.pr.modify(|_, w| w.pr12().set_bit());
        //look if ws/pb1 is high
        if ws {
            //disable interrupt on EXTI12
            exti.imr.modify(|_, w| w.mr12().clear_bit());
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
            rprintln!("Resynced (EXTI0)");
        }
    }
    //unused interrupt for sofware task
    extern "C" {
        fn EXTI0();
        fn EXTI1();
        fn EXTI2();
        fn EXTI3();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
