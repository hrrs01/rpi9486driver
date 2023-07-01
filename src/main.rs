#![no_std]
#![no_main]

use driver::DisplayDriver;
use embedded_hal::digital::v2::ToggleableOutputPin;
use pac::SPI1;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::Clock;
use rp_pico::hal::Spi;
use rp_pico::hal::gpio;
use rp_pico::hal::spi::Enabled;
use rp_pico::pac;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use fugit::RateExtU32;

use crate::driver::Interface;

mod driver;
mod spiv2;



#[entry]
fn main() -> ! {
    // rtt_init_print!();
    // rprintln!("Starting program...");
    
    // We need this, else the problem should panic.
    let mut pac = pac::Peripherals::take().unwrap();
    // Grabbing the core peripheral from the PAC.
    let core = pac::CorePeripherals::take().unwrap();

    // Set up a watchdog, to monitor that we dont get stuck in a loop.
    // We are creating a Watchdog object by grabbing the watchdog from the pac and putting it into a object.
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Create a delay source with the core system timer, and the system clock frequency.
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins.
    let sio = hal::Sio::new(pac.SIO);

    // Create a variable containing all the pins, already mapped for us according to the Pico documentation.
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS
    );

    // Configuring SPI pins.
    let mut _sck_pin = pins.gpio10.into_mode::<gpio::FunctionSpi>();
    let mut _mosi_pin = pins.gpio11.into_mode::<gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio13.into_push_pull_output();
    cs_pin.set_high().unwrap();

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    // Data/clock switch pin
    let mut dc_pin = pins.gpio7.into_push_pull_output();
    dc_pin.set_high().unwrap();

    let mut rst_pin = pins.gpio8.into_push_pull_output();
    rst_pin.set_low().unwrap();


    // SPI configuration
    let mut spi = Spi::<_,_,16>::new(pac.SPI1);
    // 20MHz is max frequency for this screen.
    let mut spi = spi.init(&mut pac.RESETS, clocks.peripheral_clock.freq(), 20.MHz(), &embedded_hal::spi::MODE_0);

    let mut interface = Interface {
        spi,
        cs_pin,
        dc_pin,
        rst_pin,
    };

    let mut driver = DisplayDriver::new(interface);
    driver.init(&mut delay);
    driver.clear_screen(0x00000);
    driver.draw_rect(269, 0, 50, 480, 0x07e0);

    loop {
        // rprintln!("Running...");
        delay.delay_ms(1000);
        led_pin.toggle().unwrap();
    }
}