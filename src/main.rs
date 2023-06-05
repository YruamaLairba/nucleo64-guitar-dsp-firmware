#![no_std]
#![no_main]

use core::default::Default;
use core::panic::PanicInfo;
use core::sync::atomic::AtomicU16;
use rtt_target::rprintln;

use arr_macro::arr;

use stm32f4xx_hal as hal;

static I2SBUF: [AtomicU16; 128] = arr![AtomicU16::new(0);128];

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true,dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use core::fmt::Write;

    use super::hal;
    use super::I2SBUF;

    use hal::dma::config::DmaConfig;
    use hal::dma::traits::{Stream, StreamISR};
    use hal::dma::{DmaChannel, DmaDirection, Stream3, Stream4, StreamX, StreamsTuple, Transfer};
    use hal::gpio::{Edge, NoPin, Output, Pin, Speed};
    use hal::i2s::stm32_i2s_v12x::driver::*;
    use hal::i2s::DualI2s;
    use hal::pac::Interrupt;
    use hal::pac::{self, DMA1, EXTI, SPI2};
    use hal::prelude::*;
    use hal::spi::{self, Spi};

    use heapless::spsc::*;
    use wm8731_another_hal::interface::SPIInterfaceU8;
    use wm8731_another_hal::prelude::*;

    use rtt_target::{rprintln, rtt_init, set_print_channel};

    type Wm8731Codec = Wm8731<SPIInterfaceU8<Spi<pac::SPI1>, Pin<'B', 2, Output>>>;
    type DualI2s2Driver = DualI2sDriver<DualI2s<SPI2>, Master, Receive, Transmit, Philips>;
    type AdcRxStream = StreamX<DMA1, 3>;
    type DacTxStream = StreamX<DMA1, 4>;

    // Part of the frame we currently transmit or receive
    #[derive(Copy, Clone)]
    pub enum FrameState {
        LeftMsb,
        LeftLsb,
        RightMsb,
        RightLsb,
    }

    use FrameState::{LeftLsb, LeftMsb, RightLsb, RightMsb};

    impl Default for FrameState {
        fn default() -> Self {
            Self::LeftMsb
        }
    }
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
        adc_p: Producer<'static, (i32, i32), 2>,
        process_c: Consumer<'static, (i32, i32), 2>,
        process_p: Producer<'static, (i32, i32), 2>,
        dac_c: Consumer<'static, (i32, i32), 2>,
    }

    #[init(local = [queue_1: Queue<(i32,i32), 2> = Queue::new(),queue_2: Queue<(i32,i32), 2> = Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let queue_1 = cx.local.queue_1;
        let queue_2 = cx.local.queue_2;
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
        let (adc_p, process_c) = queue_1.split();
        let (process_p, dac_c) = queue_2.split();
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
        i2s2_driver.ext().set_tx_interrupt(true);
        i2s2_driver.ext().set_tx_dma(true);
        i2s2_driver.ext().set_error_interrupt(true);

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
        adc_rx_stream.set_number_of_transfers(I2SBUF.len() as u16);
        unsafe {
            adc_rx_stream.set_memory_size(1);
            adc_rx_stream.set_peripheral_size(1);
        };
        adc_rx_stream.set_memory_increment(true);
        adc_rx_stream.set_circular_mode(true);
        adc_rx_stream.set_direction(DmaDirection::PeripheralToMemory);
        adc_rx_stream.set_interrupts_enable(true, true, true, true);
        dac_tx_stream.set_channel(DmaChannel::Channel2);
        dac_tx_stream.set_peripheral_address(i2s2_driver.main().data_register_address());
        dac_tx_stream.set_memory_address(&I2SBUF as *const _ as u32);
        dac_tx_stream.set_number_of_transfers(I2SBUF.len() as u16);
        unsafe {
            dac_tx_stream.set_memory_size(1);
            dac_tx_stream.set_peripheral_size(1);
        };
        dac_tx_stream.set_memory_increment(false);
        dac_tx_stream.set_circular_mode(true);
        dac_tx_stream.set_direction(DmaDirection::MemoryToPeripheral);
        dac_tx_stream.set_interrupts_enable(true, true, true, true);

        unsafe { adc_rx_stream.enable() };
        unsafe { dac_tx_stream.enable() };
        i2s2_driver.main().enable();

        (
            Shared {
                wm8731,
                i2s2_driver,
                adc_rx_stream,
                dac_tx_stream,
                exti,
            },
            Local {
                logs_chan,
                adc_p,
                process_c,
                process_p,
                dac_c,
            },
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
    fn log(cx: log::Context, message: &'static str) {
        writeln!(cx.local.logs_chan, "{}", message).unwrap();
    }

    // processing audio
    #[task(binds = SPI5, local = [count: u32 = 0,process_c,process_p])]
    fn process(cx: process::Context) {
        let count = cx.local.count;
        let process_c = cx.local.process_c;
        let process_p = cx.local.process_p;
        while let Some(mut smpl) = process_c.dequeue() {
            let period = 24000;
            if *count > period / 2 {
                smpl.0 >>= 1;
            }
            if *count > period / 4 && *count <= period * 3 / 4 {
                smpl.1 >>= 1;
            }
            *count += 1;
            if *count >= period {
                *count = 0;
            }
            process_p.enqueue(smpl).ok();
        }
    }

    #[task(
        priority = 4,
        binds = SPI2,
        local = [
            main_frame_state: FrameState = LeftMsb,
            main_frame: (u32,u32) = (0,0),
            ext_frame_state: FrameState = LeftMsb,
            ext_frame: (u32,u32) = (0,0),
            adc_p,
            dac_c
        ],
        shared = [i2s2_driver, exti]
    )]
    fn i2s2(cx: i2s2::Context) {
        let i2s2_driver = cx.shared.i2s2_driver;

        // handling "main" part
        let main_frame_state = cx.local.main_frame_state;
        let main_frame = cx.local.main_frame;
        let adc_p = cx.local.adc_p;
        let status = i2s2_driver.main().status();
        // It's better to read first to avoid triggering ovr flag
        if status.rxne() {
            let data = i2s2_driver.main().read_data_register();
            match (*main_frame_state, status.chside()) {
                (LeftMsb, Channel::Left) => {
                    main_frame.0 = (data as u32) << 16;
                    *main_frame_state = LeftLsb;
                }
                (LeftLsb, Channel::Left) => {
                    main_frame.0 |= data as u32;
                    *main_frame_state = RightMsb;
                }
                (RightMsb, Channel::Right) => {
                    main_frame.1 = (data as u32) << 16;
                    *main_frame_state = RightLsb;
                }
                (RightLsb, Channel::Right) => {
                    main_frame.1 |= data as u32;
                    // defer sample processing to another task
                    let (l, r) = *main_frame;
                    adc_p.enqueue((l as i32, r as i32)).ok();
                    rtic::pend(Interrupt::SPI5);
                    *main_frame_state = LeftMsb;
                }
                // in case of ovr this resynchronize at start of new main_frame
                _ => *main_frame_state = LeftMsb,
            }
        }
        if status.ovr() {
            log::spawn("i2s2 Overrun").ok();
            // sequence to delete ovr flag
            i2s2_driver.main().read_data_register();
            i2s2_driver.main().status();
        }

        // handling "ext" part
        let ext_frame_state = cx.local.ext_frame_state;
        let ext_frame = cx.local.ext_frame;
        let dac_c = cx.local.dac_c;
        let exti = cx.shared.exti;
        let status = i2s2_driver.ext().status();
        // it's better to write data first to avoid to trigger udr flag
        if status.txe() {
            let data;
            match (*ext_frame_state, status.chside()) {
                (LeftMsb, Channel::Left) => {
                    let (l, r) = dac_c.dequeue().unwrap_or_default();
                    *ext_frame = (l as u32, r as u32);
                    data = (ext_frame.0 >> 16) as u16;
                    *ext_frame_state = LeftLsb;
                }
                (LeftLsb, Channel::Left) => {
                    data = (ext_frame.0 & 0xFFFF) as u16;
                    *ext_frame_state = RightMsb;
                }
                (RightMsb, Channel::Right) => {
                    data = (ext_frame.1 >> 16) as u16;
                    *ext_frame_state = RightLsb;
                }
                (RightLsb, Channel::Right) => {
                    data = (ext_frame.1 & 0xFFFF) as u16;
                    *ext_frame_state = LeftMsb;
                }
                // in case of udr this resynchronize tracked and actual channel
                _ => {
                    *ext_frame_state = LeftMsb;
                    data = 0; //garbage data to avoid additional underrun
                }
            }
            i2s2_driver.ext().write_data_register(data);
        }
        if status.fre() {
            log::spawn("i2s2 Frame error").ok();
            i2s2_driver.ext().disable();
            i2s2_driver.ws_pin_mut().enable_interrupt(exti);
        }
        if status.udr() {
            log::spawn("i2s2 udr").ok();
            i2s2_driver.ext().status();
            i2s2_driver.ext().write_data_register(0);
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
            i2s2_driver.ext().write_data_register(0);
            i2s2_driver.ext().enable();
        }
    }

    #[task(priority = 4, binds = DMA1_STREAM4, shared = [dac_tx_stream])]
    fn dma1_stream4(cx: dma1_stream4::Context) {
        let dac_tx_stream = cx.shared.dac_tx_stream;
        if DacTxStream::get_transfer_complete_flag() {
            log::spawn("dac_tx_stream transfert complete").ok();
        }
        if DacTxStream::get_half_transfer_flag() {
            log::spawn("dac_tx_stream half transfert complete").ok();
        }
        if DacTxStream::get_transfer_error_flag() {
            log::spawn("dac_tx_stream transfert error").ok();
        }
        if DacTxStream::get_fifo_error_flag() {
            log::spawn("dac_tx_stream fifo error").ok();
        }
        if DacTxStream::get_direct_mode_error_flag() {
            log::spawn("dac_tx_stream direct mode error").ok();
        }
        dac_tx_stream.clear_interrupts();
    }

    #[task(priority = 4, binds = DMA1_STREAM3, shared = [adc_rx_stream])]
    fn dma1_stream3(cx: dma1_stream3::Context) {
        let adc_rx_stream = cx.shared.adc_rx_stream;
        if AdcRxStream::get_transfer_complete_flag() {
            log::spawn("adc_rx_stream transfert complete").ok();
        }
        if AdcRxStream::get_half_transfer_flag() {
            log::spawn("adc_rx_stream half transfert complete").ok();
        }
        if AdcRxStream::get_transfer_error_flag() {
            log::spawn("adc_rx_stream transfert error").ok();
        }
        if AdcRxStream::get_fifo_error_flag() {
            log::spawn("adc_rx_stream fifo error").ok();
        }
        if AdcRxStream::get_direct_mode_error_flag() {
            log::spawn("adc_rx_stream direct mode error").ok();
        }
        adc_rx_stream.clear_interrupts();
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
