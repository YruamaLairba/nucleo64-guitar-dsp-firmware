//! examples/init.rs

#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

use core::panic::PanicInfo;
use cortex_m_semihosting::debug;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{stm32};
use stm32;
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

#[rtic::app(device = stm32f4::stm32f411, peripherals = true)]
const APP: () = {
    #[init]
    fn init(_: init::Context) {
        rtt_init_print!();
        rprintln!("init");
    let device = stm32::Peripherals::take().unwrap();
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



    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        static mut X: u32 = 0;

        // Safe access to local `static mut` variable
        let _x: &'static mut u32 = X;

        rprintln!("idle");

        debug::exit(debug::EXIT_SUCCESS);

        loop {}
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
