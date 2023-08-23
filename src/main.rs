#![no_std]
#![no_main]

use core::panic::PanicInfo;
use core::sync::atomic::AtomicU16;
use rtt_target::rprintln;

use arr_macro::arr;

use stm32f4xx_hal as hal;

const I2SBUFSIZE: usize = 256;
static I2SBUF: [[AtomicU16; I2SBUFSIZE / 2]; 2] =
    [arr![AtomicU16::new(0);128], arr![AtomicU16::new(0);128]];

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true,dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use core::fmt::Write;
    use core::sync::atomic::AtomicU16;
    use core::sync::atomic::Ordering::*;

    use super::hal;
    use super::{I2SBUF, I2SBUFSIZE};

    use hal::dma::traits::{Stream, StreamISR};
    use hal::dma::{
        DmaChannel, DmaCommonInterrupts, DmaDataSize::*, DmaDirection, Stream3, Stream4,
        StreamsTuple,
    };
    use hal::gpio::{Edge, NoPin, Output, Pin, Speed};
    use hal::i2s::stm32_i2s_v12x::driver::*;
    use hal::i2s::DualI2s;
    use hal::pac::{self, DMA1, EXTI, SPI2};
    use hal::prelude::*;
    use hal::spi::{self, Spi};

    use wm8731_another_hal::interface::SPIInterfaceU8;
    use wm8731_another_hal::prelude::*;

    use rtt_target::{rprintln, rtt_init, set_print_channel};

    type Wm8731Codec = Wm8731<SPIInterfaceU8<Spi<pac::SPI1>, Pin<'B', 2, Output>>>;
    type DualI2s2Driver = DualI2sDriver<DualI2s<SPI2>, Master, Receive, Transmit, Philips>;

    // Part of the frame we currently transmit or receive
    #[derive(Copy, Clone)]
    pub enum FrameState {
        LeftMsb,
        LeftLsb,
        RightMsb,
        RightLsb,
    }

    impl Default for FrameState {
        fn default() -> Self {
            Self::LeftMsb
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub enum I2sErr {
        I2sMainOvr,
        I2sMainUdr,
        I2sMainFre,
        I2sExtOvr,
        I2sExtUdr,
        I2sExtFre,
    }
    use I2sErr::*;

    #[derive(Debug, Clone, Copy)]
    pub enum Log {
        I2sError(I2sErr),
        DacTxStreamTransferComplete(u16),
        DacTxStreamHalfTransfer(u16),
        DacTxStreamTransferError,
        DacTxStreamFifoError(u16, I2sErr),
        DacTxStreamDirectModeError,
        AdcRxStreamTransferComplete(u16),
        AdcRxStreamHalfTransfer(u16),
        AdcRxStreamTransferError,
        AdcRxStreamFifoError(u16, I2sErr),
        AdcRxStreamDirectModeError,
    }
    use Log::*;

    #[shared]
    struct Shared {
        wm8731: Wm8731Codec,
        #[lock_free]
        i2s2_driver: DualI2s2Driver,
        #[lock_free]
        adc_rx_stream: Stream3<DMA1>,
        #[lock_free]
        dac_tx_stream: Stream4<DMA1>,
        #[lock_free]
        exti: EXTI,
    }

    #[local]
    struct Local {
        logs_chan: rtt_target::UpChannel,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let channels = rtt_init! {
            up: {
                0: {
                    size: 128
                    name: "Logs"
                }
                1: {
                    size: 1024
                    name: "Panics"
                }
            }
        };
        let logs_chan = channels.up.0;
        let panics_chan = channels.up.1;
        set_print_channel(panics_chan);
        let device = cx.device;
        let mut syscfg = device.SYSCFG.constrain();
        let mut exti = device.EXTI;
        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let rcc = device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8u32.MHz())
            .sysclk(96.MHz())
            .hclk(96.MHz())
            .pclk1(50.MHz())
            .pclk2(100.MHz())
            .i2s_clk(61440.kHz())
            .freeze();

        //Spi com
        let pa5 = gpioa.pa5; //CK
        let pa7 = gpioa.pa7; //MOSI
        let pb2 = gpiob.pb2.into_push_pull_output(); //CS
                                                     //let _ = pb2.set_high();

        let spi1_mode = spi::Mode {
            polarity: spi::Polarity::IdleHigh,
            phase: spi::Phase::CaptureOnSecondTransition, //With IdleHigh, capture on rising edge
        };

        let spi1 = Spi::new(
            device.SPI1,
            (pa5, NoPin::new(), pa7),
            spi1_mode,
            500.kHz(),
            &clocks,
        );

        rprintln!("I2SBUF{:?}", &I2SBUF);

        let mut wm8731 = Wm8731::new(SPIInterfaceU8::new(spi1, pb2));
        {
            //power down
            rprintln!("Power Down");
            wm8731.set_lineinpd(false);
            wm8731.set_micpd(false);
            wm8731.set_adcpd(false);
            wm8731.set_dacpd(false);
            wm8731.set_oscpd(false);
            wm8731.set_clkoutpd(false);
            wm8731.set_poweroff(false);
            rprintln!("Mute headphone");
            wm8731.set_both_hpvol(HpVoldB::MUTE, false);
            rprintln!("Unmute line in");
            wm8731.set_both_inmute(false);
            wm8731.set_both_invol(InVoldB::Z0DB);
            rprintln!("Anaoutput Path");
            wm8731.set_micboost(false);
            wm8731.set_mutemic(true);
            wm8731.set_insel(InselV::Line);
            wm8731.set_bypass(false);
            wm8731.set_dacsel(true);
            wm8731.set_sidetone(false);
            //digital_audio_path
            //wm8731.set_adchpd(false);
            wm8731.set_dacmu(false);
            //wm8731.set_deemp(false);
            //digital_audio_interface
            wm8731.set_format(FormatV::I2s);
            wm8731.set_iwl(IwlV::Iwl16Bits);
            wm8731.set_lrp(false);
            wm8731.set_lrswap(false);
            wm8731.set_ms(MsV::Slave);
            wm8731.set_bclkinv(false);
            //sampling
            wm8731.set_sampling_rates(SamplingRates::ADC256_DAC256_A);
            wm8731.set_clkidiv2(false);
            wm8731.set_clkodiv2(false);
            rprintln!("Out power up");
            wm8731.set_outpd(false);
            let hpvol = HpVoldB::N18DB;
            rprintln!("Setting HP vol to {}", hpvol);
            wm8731.set_both_hpvol(HpVoldB::N18DB, true);
        }
        wm8731.activate();

        // Workaround for corrupted last bit of data issue, see stm32f411 errata
        let mut pb13 = gpiob.pb13.into_alternate::<5>();
        pb13.set_speed(Speed::VeryHigh);

        // I2S pins: (WS, CK, MCLK, SD) for I2S2
        let i2s2_pins = (
            gpiob.pb12, //WS
            pb13,       //CK
            gpioc.pc6,  //MCK
            gpiob.pb15, //SD
            gpiob.pb14, //ExtSD
        );
        let i2s2 = DualI2s::new(device.SPI2, device.I2S2EXT, i2s2_pins, &clocks);
        let i2s2_config = DualI2sDriverConfig::new_master()
            .direction(Receive, Transmit)
            .standard(Philips)
            .data_format(DataFormat::Data24Channel32)
            .master_clock(true)
            .request_frequency(48_000);
        let mut i2s2_driver = DualI2sDriver::new(i2s2, i2s2_config);
        rprintln!("actual sample rate is {}", i2s2_driver.sample_rate());
        i2s2_driver.main().set_rx_interrupt(true);
        i2s2_driver.main().set_rx_dma(true);
        i2s2_driver.main().set_error_interrupt(true);
        i2s2_driver.ext().set_tx_interrupt(false);
        i2s2_driver.ext().set_tx_dma(true);
        i2s2_driver.ext().set_error_interrupt(false);

        // set up an interrupt on WS pin
        let ws_pin = i2s2_driver.ws_pin_mut();
        ws_pin.make_interrupt_source(&mut syscfg);
        ws_pin.trigger_on_edge(&mut exti, Edge::Rising);
        // we will enable the ext part in interrupt
        ws_pin.enable_interrupt(&mut exti);

        // dma setup
        let streams = StreamsTuple::new(device.DMA1);
        let mut adc_rx_stream = streams.3;
        let mut dac_tx_stream = streams.4;

        adc_rx_stream.set_channel(DmaChannel::Channel0);
        adc_rx_stream.set_peripheral_address(i2s2_driver.main().data_register_address());
        adc_rx_stream.set_memory_address(&I2SBUF as *const _ as u32);
        adc_rx_stream.set_number_of_transfers(I2SBUFSIZE as u16);
        unsafe {
            adc_rx_stream.set_memory_size(HalfWord);
            adc_rx_stream.set_peripheral_size(HalfWord);
        };
        adc_rx_stream.set_memory_increment(true);
        adc_rx_stream.set_circular_mode(true);
        adc_rx_stream.set_direction(DmaDirection::PeripheralToMemory);
        adc_rx_stream.listen(DmaCommonInterrupts {
            transfer_complete: true,
            half_transfer: true,
            transfer_error: true,
            direct_mode_error: true,
        });
        dac_tx_stream.set_channel(DmaChannel::Channel2);
        dac_tx_stream.set_peripheral_address(i2s2_driver.ext().data_register_address());
        dac_tx_stream.set_memory_address(&I2SBUF as *const _ as u32);
        dac_tx_stream.set_number_of_transfers(I2SBUFSIZE as u16);
        unsafe {
            dac_tx_stream.set_memory_size(HalfWord);
            dac_tx_stream.set_peripheral_size(HalfWord);
        };
        dac_tx_stream.set_memory_increment(true);
        dac_tx_stream.set_circular_mode(true);
        dac_tx_stream.set_direction(DmaDirection::MemoryToPeripheral);
        dac_tx_stream.listen(DmaCommonInterrupts {
            transfer_complete: false,
            half_transfer: false,
            transfer_error: true,
            direct_mode_error: true,
        });

        //unsafe { adc_rx_stream.enable() };
        //unsafe { dac_tx_stream.enable() };
        i2s2_driver.main().enable();

        (
            Shared {
                wm8731,
                i2s2_driver,
                adc_rx_stream,
                dac_tx_stream,
                exti,
            },
            Local { logs_chan },
            init::Monotonics(),
        )
    }

    #[idle(shared = [], local = [])]
    fn idle(_cx: idle::Context) -> ! {
        #[allow(clippy::empty_loop)]
        loop {}
    }

    // Printing message directly in a i2s interrupt can cause timing issues.
    #[task(capacity = 10, local = [logs_chan])]
    fn log(cx: log::Context, log: Log) {
        writeln!(cx.local.logs_chan, "{:?}", log).unwrap();
    }

    // processing audio
    #[task]
    fn process(_cx: process::Context, data: &'static [AtomicU16; I2SBUFSIZE / 2]) {
        rprintln!("process");
        let data_iter = data.chunks(4);
        for e in data_iter {
            let l_msb = e[0].load(Relaxed);
            let l_lsb = e[1].load(Relaxed);
            let r_msb = e[2].load(Relaxed);
            let r_lsb = e[3].load(Relaxed);
            let mut smpl = (
                ((l_msb as u32) << 16) + (l_lsb as u32),
                ((r_msb as u32) << 16) + (r_lsb as u32),
            );
            smpl.1 = 0;

            let l_msb = (smpl.0 >> 16) as u16;
            let l_lsb = (smpl.0 & 0x0000_FFFF) as u16;
            let r_msb = (smpl.1 >> 16) as u16;
            let r_lsb = (smpl.1 & 0x0000_FFFF) as u16;
            e[0].store(l_msb, Release);
            e[1].store(l_lsb, Release);
            e[2].store(r_msb, Release);
            e[3].store(r_lsb, Release);
        }
    }

    #[task(
        priority = 4,
        binds = SPI2,
        shared = [i2s2_driver,adc_rx_stream, dac_tx_stream, exti]
    )]
    fn i2s2(cx: i2s2::Context) {
        let i2s2_driver = cx.shared.i2s2_driver;
        let adc_rx_stream = cx.shared.adc_rx_stream;
        let dac_tx_stream = cx.shared.dac_tx_stream;
        let exti = cx.shared.exti;

        // handling "main" part
        let status = i2s2_driver.main().status();
        // rxne event is just used to start the dma stream
        if status.rxne() {
            //i2s2_driver.main().read_data_register();
            unsafe { adc_rx_stream.enable() };
            i2s2_driver.main().set_rx_interrupt(false);
        }
        if status.ovr() {
            log::spawn(I2sError(I2sMainOvr)).ok();
            // stops dma streams
            unsafe {
                adc_rx_stream.disable();
                dac_tx_stream.disable();
            }
            while adc_rx_stream.is_enabled() || dac_tx_stream.is_enabled() {}
            // sequence to delete ovr flag
            i2s2_driver.main().read_data_register();
            i2s2_driver.main().status();
            // disable extension first to avoid triggering error when resetting clocks
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);

            i2s2_driver.main().disable();
            i2s2_driver.reset_clocks();
            i2s2_driver.main().enable();
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }

        // handling "ext" part
        let status = i2s2_driver.ext().status();
        // txe event is just used to start the dma stream
        if status.txe() {
            let nb_transfers = adc_rx_stream.number_of_transfers();
            if nb_transfers == 1 {
                unsafe { dac_tx_stream.enable() };
                i2s2_driver.ext().set_tx_interrupt(false);
            } else {
                i2s2_driver.ext().write_data_register(0);
            }
        }
        if status.fre() {
            log::spawn(I2sError(I2sExtFre)).ok();
            unsafe {
                dac_tx_stream.disable();
            }
            while dac_tx_stream.is_enabled() {}
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }
        if status.udr() {
            log::spawn(I2sError(I2sExtUdr)).ok();
            unsafe {
                dac_tx_stream.disable();
            }
            while dac_tx_stream.is_enabled() {}
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }
    }

    // Look WS line for the "ext" part (re) synchronisation
    #[task(priority = 4, binds = EXTI15_10, shared = [i2s2_driver,exti])]
    fn exti15_10(cx: exti15_10::Context) {
        let i2s2_driver = cx.shared.i2s2_driver;
        let exti = cx.shared.exti;
        let ws_pin = i2s2_driver.ws_pin_mut();
        // check if that pin triggered the interrupt
        if ws_pin.check_interrupt() {
            // Here we know ws pin is high because the interrupt was triggerd by it's rising edge
            ws_pin.clear_interrupt_pending_bit();
            ws_pin.disable_interrupt(exti);
            i2s2_driver.ext().set_tx_interrupt(true);
            i2s2_driver.ext().set_error_interrupt(true);

            i2s2_driver.ext().write_data_register(0);
            i2s2_driver.ext().enable();
        }
    }

    #[task(priority = 4, binds = DMA1_STREAM4, shared = [dac_tx_stream,i2s2_driver,exti])]
    fn dma1_stream4(cx: dma1_stream4::Context) {
        let dac_tx_stream = cx.shared.dac_tx_stream;
        let i2s2_driver = cx.shared.i2s2_driver;
        let exti = cx.shared.exti;
        let nb_transfers = dac_tx_stream.number_of_transfers();
        let flags = dac_tx_stream.all_flags();
        let clear_flags = hal::dma::DmaFlags {
            transfer_complete: true,
            half_transfer: true,
            transfer_error: true,
            direct_mode_error: true,
            fifo_error: true,
        };
        dac_tx_stream.clear_flags(clear_flags);
        if flags.transfer_complete {
            //log::spawn(DacTxStreamTransferComplete(nb_transfers)).ok();
        }
        if flags.half_transfer {
            //log::spawn(DacTxStreamHalfTransfer(nb_transfers)).ok();
        }
        // this flag can only set on a bus error but i don't know what it mean
        if flags.transfer_error {
            log::spawn(DacTxStreamTransferError).ok();
            unsafe {
                dac_tx_stream.disable();
            }
            while dac_tx_stream.is_enabled() {}
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }
        // this error can only happen in underrun condition with current stream condition
        if flags.fifo_error {
            log::spawn(DacTxStreamFifoError(nb_transfers, I2sExtUdr)).ok();
            unsafe {
                dac_tx_stream.disable();
            }
            while dac_tx_stream.is_enabled() {}
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }
        // this error can't happen with this current stream configuration.
        if flags.direct_mode_error {
            log::spawn(DacTxStreamDirectModeError).ok();
        }
    }

    #[task(priority = 4, binds = DMA1_STREAM3, shared = [adc_rx_stream,dac_tx_stream,i2s2_driver,exti])]
    fn dma1_stream3(cx: dma1_stream3::Context) {
        let adc_rx_stream = cx.shared.adc_rx_stream;
        let dac_tx_stream = cx.shared.dac_tx_stream;
        let i2s2_driver = cx.shared.i2s2_driver;
        let exti = cx.shared.exti;
        let nb_transfers = adc_rx_stream.number_of_transfers();
        let flags = adc_rx_stream.all_flags();
        let clear_flags = hal::dma::DmaFlags {
            transfer_complete: true,
            half_transfer: true,
            transfer_error: true,
            direct_mode_error: true,
            fifo_error: true,
        };
        adc_rx_stream.clear_flags(clear_flags);

        let flags2 = adc_rx_stream.all_flags();
        if flags2.transfer_complete
            || flags2.half_transfer
            || flags2.transfer_error
            || flags2.direct_mode_error
            || flags2.fifo_error
        {
            rprintln!("oh crap {:?}", flags2);
        }

        if flags.transfer_complete {
            //log::spawn(AdcRxStreamTransferComplete(nb_transfers)).ok();
            process::spawn(&I2SBUF[1]).ok();
        }
        if flags.half_transfer {
            //log::spawn(AdcRxStreamHalfTransfer(nb_transfers)).ok();
            process::spawn(&I2SBUF[0]).ok();
        }

        // this flag can only set on a bus error but i don't know what it mean
        if flags.transfer_error {
            log::spawn(AdcRxStreamTransferError).ok();
            // stops dma streams
            unsafe {
                adc_rx_stream.disable();
                dac_tx_stream.disable();
            }
            while adc_rx_stream.is_enabled() || dac_tx_stream.is_enabled() {}
            // sequence to delete ovr flag
            i2s2_driver.main().read_data_register();
            i2s2_driver.main().status();
            // disable extension first to avoid triggering error when resetting clocks
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);

            i2s2_driver.main().disable();
            i2s2_driver.reset_clocks();
            i2s2_driver.main().enable();
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }

        // this error can only happen in overrun condition with current stream condition
        if flags.fifo_error {
            log::spawn(AdcRxStreamFifoError(nb_transfers, I2sExtUdr)).ok();
            // stops dma streams
            unsafe {
                adc_rx_stream.disable();
                dac_tx_stream.disable();
            }
            while adc_rx_stream.is_enabled() || dac_tx_stream.is_enabled() {}
            // sequence to delete ovr flag
            i2s2_driver.main().read_data_register();
            i2s2_driver.main().status();
            // disable extension first to avoid triggering error when resetting clocks
            i2s2_driver.ext().disable();
            i2s2_driver.ext().set_tx_interrupt(true);

            i2s2_driver.main().disable();
            i2s2_driver.reset_clocks();
            i2s2_driver.main().enable();
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }
        // this error can't happen with this current stream configuration.
        if flags.direct_mode_error {
            log::spawn(AdcRxStreamDirectModeError).ok();
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
