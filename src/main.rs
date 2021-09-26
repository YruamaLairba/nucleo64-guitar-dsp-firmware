//! examples/init.rs

//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem::size_of;
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
use stm32f4xx_hal::stm32::{DMA1, EXTI, I2S2EXT, SPI1, SPI2};
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

const BUF_SIZE: usize = 64;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        spi2: SPI2,
        i2s2ext: I2S2EXT,
        dma1: DMA1,
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
            dma1,
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

    #[task(binds = SPI2,  resources = [spi2,i2s2ext,dma1,exti,wm8731,buf],spawn = [process])]
    fn spi2(cx: spi2::Context) {
        static mut PREVIOUS_ADC_SIDE: Option<CHSIDE_A> = None;
        static mut PREVIOUS_DAC_SIDE: Option<CHSIDE_A> = None;
        //static mut ADC_DATA: [u16; 4] = [0; 4];
        //static mut DAC_DATA: [u16; 4] = [0; 4];
        //static mut I2SEXT_FRE: bool = false;

        let spi2 = cx.resources.spi2;
        let i2s2ext = cx.resources.i2s2ext;
        let dma1 = cx.resources.dma1;
        let exti = cx.resources.exti;
        let wm8731 = cx.resources.wm8731;
        let buf = cx.resources.buf;
        //if *COUNT >= 48000 *4 {
        //    *COUNT=0;
        //    rprintln!("i2s i");
        //}
        //*COUNT += 1;
        let dac_transfer = dma1.st[4].ndtr.read().bits() as usize;
        let spi2_sr_read = spi2.sr.read();
        let i2sext_sr_read = i2s2ext.sr.read();

        if i2sext_sr_read.txe().bit() {
            let side = i2sext_sr_read.chside().variant();
            if dma1.st[4].cr.read().en().is_disabled() {
                //Write garbage to avoid undesirable underrun interrupt
                i2s2ext.dr.write(|w| w.dr().bits(0));
                if *PREVIOUS_DAC_SIDE == Some(CHSIDE_A::RIGHT) && side == CHSIDE_A::RIGHT {
                    i2s2ext.cr2.modify(|_, w| w.txeie().clear_bit());
                    dma1.st[4].cr.modify(|_, w| w.en().enabled());
                }
            }
            *PREVIOUS_DAC_SIDE = Some(side);
        }

        if spi2_sr_read.rxne().bit() {
            let side = spi2_sr_read.chside().variant();
            if dma1.st[3].cr.read().en().is_disabled() {
                //Read data to avoid undesirable overrun interrupt
                spi2.dr.read().dr().bits();
                let nb_transfer = buf.len() * size_of::<I2sSample>() / 2;
                //ensure ADC dma start just after the DAC dma
                if *PREVIOUS_ADC_SIDE == Some(CHSIDE_A::RIGHT)
                    && side == CHSIDE_A::RIGHT
                    && dac_transfer <= (nb_transfer - 1)
                    && dac_transfer >= (nb_transfer - 4)
                {
                    spi2.cr2.modify(|_, w| w.rxneie().clear_bit());
                    dma1.st[3].cr.modify(|_, w| w.en().enabled());
                }
            }
            *PREVIOUS_ADC_SIDE = Some(side);
        }

        //error management
        let mut dac_not_sync = false;
        let mut adc_not_sync = false;
        if spi2_sr_read.fre().bit() || spi2_sr_read.ovr().bit() || spi2_sr_read.udr().bit() {
            if spi2_sr_read.fre().bit() {
                //can never happen in master mode
                rprintln!("i2s Frame Error");
            }
            if spi2_sr_read.ovr().bit() {
                rprintln!("i2s Overrun");
            }
            if spi2_sr_read.udr().bit() {
                //can only happen in slave transmission mode
                rprintln!("i2s underrun");
            }
            //clear error flags
            spi2.dr.read();
            spi2.sr.read();
            adc_not_sync = true;
        }
        if i2sext_sr_read.fre().bit() {
            rprintln!("i2sext Frame Error");
            //resynchronization
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().disabled());
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
            //let gpiob = &(*stm32::GPIOB::ptr());
            //let ws = gpiob.idr.read().idr12().bit();
            //if ws {
            //    rprintln!("Resynced (I2S2EXT)");
            //}
            //else {
            //    rprintln!("ooops");
            //    let exti = &(*stm32::EXTI::ptr());
            //    exti.imr.modify(|_, w| w.mr12().set_bit());
            //}
            dac_not_sync = true;
        }
        if i2sext_sr_read.ovr().bit() {
            rprintln!("i2sext Overrun");
            //this sequence reset the interrupt
            i2s2ext.dr.read();
            i2s2ext.sr.read();
            dac_not_sync = true;
        }
        if i2sext_sr_read.udr().bit() {
            rprintln!("i2sext underrun");
            //reset interrupt and reset i2sext
            i2s2ext.sr.read();
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().disabled());
            i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
            dac_not_sync = true;
        }
        if dac_not_sync {
            //adc dma need resynchronized
            adc_not_sync = true;
            i2s2ext.cr2.modify(|_, w| w.txeie().set_bit());
            dma1.st[4].cr.modify(|_, w| w.en().disabled());
        }
        if adc_not_sync {
            spi2.cr2.modify(|_, w| w.rxneie().set_bit());
            dma1.st[3].cr.modify(|_, w| w.en().disabled());
        }
    }

    #[task(binds = DMA1_STREAM3,resources = [dma1,spi2])]
    fn dma1_stream3(cx: dma1_stream3::Context) {
        let dma1 = cx.resources.dma1;
        let spi2 = cx.resources.spi2;
        let _remain = dma1.st[3].ndtr.read().bits();
        let lisr_read = dma1.lisr.read();
        if lisr_read.tcif3().bit_is_set() {
            //TODO transmit upper half buf to process
            rprintln!("Transfer complete");
        };
        if lisr_read.htif3().bit_is_set() {
            //TODO transmit lower half buf to process
        };
        if lisr_read.teif3().bit_is_set()
            || lisr_read.dmeif3().bit_is_set()
            || lisr_read.feif3().bit_is_set()
        {
            if lisr_read.teif3().bit_is_set() {
                rprintln!("DMA1 Stream3 transfer error")
            };
            if lisr_read.dmeif3().bit_is_set() {
                rprintln!("DMA1 Stream3 direct mode error")
            };
            if lisr_read.feif3().bit_is_set() {
                rprintln!("DMA1 Stream3 fifo error")
            };
            spi2.cr2.modify(|_, w| w.rxneie().set_bit());
            dma1.st[3].cr.modify(|_, w| w.en().disabled());
        }
        //clear all interrupt flag
        dma1.lifcr.write(|w| {
            w.ctcif3().set_bit();
            w.chtif3().set_bit();
            w.cteif3().set_bit();
            w.cdmeif3().set_bit();
            w.cfeif3().set_bit()
        });
    }

    #[task(binds = DMA1_STREAM4,resources = [dma1,i2s2ext])]
    fn dma1_stream4(cx: dma1_stream4::Context) {
        //rprintln!("DMA1_STREAM3");
        let dma1 = cx.resources.dma1;
        let i2s2ext = cx.resources.i2s2ext;
        //let remain = dma1.st[4].ndtr.read().bits();
        let hisr_read = dma1.hisr.read();
        if hisr_read.tcif4().bit_is_set() {
            //rprintln!("DMA1 Stream4 transfert complete, Remain {}", remain);
            rprintln!("st4 complete");
        }
        if hisr_read.htif4().bit_is_set() {
            //rprintln!("DMA1 Stream4 half transfert complete, Remain {}", remain);
        };
        if hisr_read.teif4().bit_is_set()
            || hisr_read.dmeif4().bit_is_set()
            || hisr_read.feif4().bit_is_set()
        {
            if hisr_read.teif4().bit_is_set() {
                rprintln!("DMA1 Stream4 transfer error")
            };
            if hisr_read.dmeif4().bit_is_set() {
                rprintln!("DMA1 Stream4 direct mode error")
            };
            if hisr_read.feif4().bit_is_set() {
                rprintln!("DMA1 Stream4 fifo error")
            };
            i2s2ext.cr2.modify(|_, w| w.txeie().set_bit());
            dma1.st[4].cr.modify(|_, w| w.en().disabled());
        }
        //clear all interrupt flag
        dma1.hifcr.write(|w| {
            w.ctcif4().set_bit();
            w.chtif4().set_bit();
            w.cteif4().set_bit();
            w.cdmeif4().set_bit();
            w.cfeif4().set_bit()
        });
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
