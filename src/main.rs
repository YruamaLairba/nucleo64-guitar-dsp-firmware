//! examples/init.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use core::panic::PanicInfo;
use cortex_m_semihosting::debug;
use rtt_target::{rprintln, rtt_init_print};
//use stm32f4::stm32f411;

#[rtic::app(device = stm32f4::stm32f411, peripherals = true)]
const APP: () = {
    #[init]
    fn init(_: init::Context) {
        rtt_init_print!();
        rprintln!("init");
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
