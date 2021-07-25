//! examples/init.rs

//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use core::panic::PanicInfo;
use cortex_m_semihosting::debug;
use rtt_target::{rprintln, rtt_init_print};
use spi::Spi;
use stm32f4xx_hal::{prelude::*, spi, stm32};
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

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    #[init]
    fn init(cx: init::Context) {
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
        //enable system clock for APB1 bus and SPI2 (I2S2)
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            rcc.apb1enr
                .modify(|_, w| w.pwren().set_bit().spi2en().set_bit());
        }
        //setup  and startup PLLI2S clock
        unsafe {
            let rcc = &(*stm32::RCC::ptr());
            //setup
            rcc.plli2scfgr.modify(|_, w| {
                w.plli2sr().bits(PLLI2SR);
                w.plli2sn().bits(PLLI2SN);
                w.plli2sm().bits(PLLI2SM)
            });
            //run the clock
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

        let mut spi1 = Spi::spi1(
            device.SPI1,
            (pa5, spi::NoMiso, pa7),
            spi1_mode,
            1.mhz().into(),
            clocks,
        );
        //Setup wm8731 chip
        //
        //Left Line in
        // 4:0 LINVOL, 10111 = 0db
        // 7 LINMUTE, 1 = mute
        // 8 LRINBOTH, 1 = apply VOL and MUTE to L & R channel
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_0000_1, 0b00010111];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Left Headphone Out
        // 6:0 HPVOL,1111001= 0db,0110000 = -73dB, 0000000 to 0101111 = MUTE
        // 7 LZCEN, enable zero cross detection
        // 8 LRHPBOTH,
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_0010_1, 0b01111001 - 12];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Analogue Audio Path Control:
        // 0 mic boost disabled: 0
        // 1 mic muted: 1
        // 2 INSEL : 0 = Line input to ADC, 1 = microphone to ADC
        // 3 BYPASS: connect Line IN to Line Out
        //
        // 4 DACSEL: connect DAC to Line Out
        // 5 SIDETONE: connect mic to line out
        // 7:6 SIDEATT: side tone attenuation (see p51)
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_0100_0, 0b0001_0010];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Digital Audi Path Control
        // 0 ADCHPD: ADC high pass enable
        // 2:1 DEEMP: de-emphasis control (see p52)
        // 3 DACMU: dac soft mute
        //
        // 4 HPOR: Store dc offset when High Pass Filter disabled (p52)
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_0101_0, 0b0000_0000];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Power down control : 1 = power down, 0 = power up
        // 0 LINEPD
        // 1 MICPD
        // 2 ADCPD
        // 3 DACPD
        //
        // 4 OUTPD
        // 5 OSCPD
        // 6 CLKOUTPD
        // 7 POWEROFF : enable power off mode
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_0110_0, 0b0110_0010];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Digital Audio Interface Format
        // 1:0 FORMAT (see p53) , I2S = 10
        // 3:2 IWL: Input audio data bit length, 32bits = 11
        //
        // 4 LRP: DACLRC phase control, to check later, default=0
        // 5 LRSWAP: DAC Left Right Clock Swap ? default=0
        // 6 MS: Master/Slave control: Slave=0
        // 7 BCLKINV: invert bit clock: Don't invert=0(default)
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_0111_0, 0b0000_1110];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Sampling Control
        // 0 USB/NORMAL : 0 = Normal
        // 1 BOSR: base oversampling 0=256fs for Normal
        // 5:2 SR: ADc and DAC sampling rate (p42-45)
        // 6 CLKDIV2: divide core clock by 2 (0= core clock = MCLK)
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_1000_0, 0b0000_0000];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Active Control
        //0 ACTIVE : 1 activate. The interface should be disabled when changing conf
        #[allow(clippy::unusual_byte_groupings)]
        let spi_buf = [0b000_1001_0, 0b0000_0001];
        let _ = pb2.set_low();
        spi1.write(&spi_buf).unwrap();
        let _ = pb2.set_high();
        //Setup i2s2 and i2s2_ext
        //gpio
        let _pb13 = gpiob.pb13.into_alternate_af5(); //CK
        let _pb15 = gpiob.pb15.into_alternate_af5(); //SD
        let _pb14 = gpiob.pb14.into_alternate_af6(); //ext_SD
        let _pb12 = gpiob.pb12.into_alternate_af5(); //WS
        let _pc6 = gpioc.pc6.into_alternate_af5(); //MCK

        //Setup an interrupt that can be triggered by pb12 pin
        //Note: The hal doesn't allow to manipulate interrupt for pin in aternate mode
        unsafe {
            let syscfg = &(*stm32::SYSCFG::ptr());
            //i on pb12
            syscfg.exticr4.modify(|_, w| w.exti12().bits(0b0001));
            let exti = &(*stm32::EXTI::ptr());
            //mask EXTI0 interrupt
            exti.imr.modify(|_, w| w.mr12().set_bit());
            //trigger interrupt on rising edge
            exti.rtsr.modify(|_, w| w.tr12().set_bit());
        };
        //i2s2 interrupt
        unsafe {
            let spi2 = &(*stm32::SPI2::ptr());
            spi2.cr2.modify(|_, w| {
                w.txeie().clear_bit();
                w.rxneie().set_bit();
                w.errie().set_bit()
            });
        }
        //i2s2_ext interrupt
        unsafe {
            let i2s2ext = &(*stm32::I2S2EXT::ptr());
            i2s2ext.cr2.modify(|_, w| {
                w.txeie().set_bit();
                w.rxneie().clear_bit();
                w.errie().set_bit()
            });
        }
        //setup spi2 peripheral into i2s mode
        unsafe {
            let spi2 = &(*stm32::SPI2::ptr());
            spi2.i2spr.modify(|_, w| {
                w.i2sdiv().bits(I2SDIV);
                w.odd().bit(ODD);
                w.mckoe().bit(MCK)
            });
            spi2.i2scfgr.modify(|_, w| {
                w.i2smod().i2smode(); //
                w.i2scfg().master_rx(); //
                w.pcmsync().long(); //
                w.i2sstd().philips(); //
                w.ckpol().idle_high(); //
                w.datlen().twenty_four_bit(); //
                w.chlen().thirty_two_bit(); //
                w.i2se().enabled()
            })
        }
        //setup i2s2ext peripheral
        unsafe {
            let i2s2ext = &(*stm32::I2S2EXT::ptr());
            i2s2ext.i2spr.modify(|_, w| {
                w.i2sdiv().bits(I2SDIV);
                w.odd().bit(ODD);
                w.mckoe().bit(MCK)
            });
            i2s2ext.i2scfgr.modify(|_, w| {
                w.i2smod().i2smode(); //
                w.i2scfg().slave_tx(); //
                w.pcmsync().long(); //
                w.i2sstd().philips(); //
                w.ckpol().idle_high(); //
                w.datlen().twenty_four_bit(); //
                w.chlen().thirty_two_bit(); //
                w.i2se().enabled()
            })
        }
        rprintln!("init done");
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        static mut X: u32 = 0;

        // Safe access to local `static mut` variable
        let _x: &'static mut u32 = X;

        rprintln!("idle");

        loop {}
    }
    #[task(binds = SPI2)]
    fn spi2(_: spi2::Context) {
        use stm32f4xx_hal::stm32::spi1::sr::CHSIDE_A;
        static mut PREVIOUS_RX_SIDE: CHSIDE_A = CHSIDE_A::RIGHT;
        static mut PREVIOUS_TX_SIDE: CHSIDE_A = CHSIDE_A::RIGHT;
        static mut RX_DATA: [u16; 4] = [0; 4];
        static mut TX_DATA: [u16; 4] = [0; 4];
        static mut SAMPLE: (u32, u32) = (0, 0);
        unsafe {
            let spi2 = &(*stm32::SPI2::ptr());
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
                if PREVIOUS_RX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::LEFT {
                    //left msb
                    RX_DATA[0] = data;
                } else if PREVIOUS_RX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::LEFT {
                    //left lsb
                    RX_DATA[1] = data;
                } else if PREVIOUS_RX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::RIGHT {
                    //right msb
                    RX_DATA[2] = data;
                } else if PREVIOUS_RX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::RIGHT {
                    //right lsb
                    RX_DATA[3] = data;
                    let left = (RX_DATA[0] as u32) << 16 | (RX_DATA[1] as u32);
                    let right = (RX_DATA[2] as u32) << 16 | (RX_DATA[3] as u32);
                    SAMPLE = (left, right);
                    //rprintln!(
                    //    "RX: {:016b} {:016b} {:016b} {:016b}",
                    //    RX_DATA[0],
                    //    RX_DATA[1],
                    //    RX_DATA[2],
                    //    RX_DATA[3]
                    //);
                }

                PREVIOUS_RX_SIDE = side;
                //rprintln!("{:b}", data);
            }

            let i2s2ext = &(*stm32::I2S2EXT::ptr());
            if i2s2ext.sr.read().fre().bit() {
                rprintln!("ext Frame Error");
                //resynchronization
                i2s2ext.i2scfgr.modify(|_, w| w.i2se().disabled());
                let gpiob = &(*stm32::GPIOB::ptr());
                let ws = gpiob.idr.read().idr12().bit();
                if ws {
                    i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
                    rprintln!("Resynced (I2S2EXT)");
                } else {
                    let exti = &(*stm32::EXTI::ptr());
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
                if PREVIOUS_TX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::LEFT {
                    //left msb
                    i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[0]));
                } else if PREVIOUS_TX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::LEFT {
                    //left lsb
                    i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[1]));
                } else if PREVIOUS_TX_SIDE == CHSIDE_A::LEFT && side == CHSIDE_A::RIGHT {
                    //right msb
                    i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[2]));
                } else if PREVIOUS_TX_SIDE == CHSIDE_A::RIGHT && side == CHSIDE_A::RIGHT {
                    //right lsb, end of audio frame
                    i2s2ext.dr.write(|w| w.dr().bits(TX_DATA[3]));
                    TX_DATA[0] = (SAMPLE.0 >> 16) as u16;
                    TX_DATA[1] = SAMPLE.0 as u16;
                    TX_DATA[2] = (SAMPLE.1 >> 16) as u16;
                    TX_DATA[3] = SAMPLE.1 as u16;
                } else {
                    unreachable!()
                };

                PREVIOUS_TX_SIDE = side;
            }
        }
    }
    #[task(binds = EXTI15_10)]
    fn exti15_10(_: exti15_10::Context) {
        unsafe {
            let gpiob = &(*stm32::GPIOB::ptr());
            let ws = gpiob.idr.read().idr12().bit();
            let exti = &(*stm32::EXTI::ptr());
            //erase the event
            exti.pr.modify(|_, w| w.pr12().set_bit());
            //look if ws/pb1 is high
            if ws {
                //disable interrupt on EXTI12
                exti.imr.modify(|_, w| w.mr12().clear_bit());
                let i2s2ext = &(*stm32::I2S2EXT::ptr());
                i2s2ext.i2scfgr.modify(|_, w| w.i2se().enabled());
                rprintln!("Resynced (EXTI0)");
            }
        }
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
