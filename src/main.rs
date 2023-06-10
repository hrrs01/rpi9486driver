#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_blocking_spi_Write;

use cortex_m::prelude::_embedded_hal_digital_OutputPin;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::Clock;
use rp_pico::hal::Spi;
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::Pin;
use rp_pico::hal::gpio::PinId;
use rp_pico::hal::gpio::PinMode;
use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::gpio::ValidPinMode;
use rp_pico::hal::spi::Enabled;
use rp_pico::hal::spi::SpiDevice;
use rp_pico::pac;

use embedded_hal::digital::v2::OutputPin;

use panic_probe as _;
use rp_pico::pac::SPI0;
use rtt_target::rprintln;
use rtt_target::rtt_init_print;

use display_interface_spi::SPIInterface;

use fugit::RateExtU32;


const ILI9468_CASET: u16 = 0x2A;
const ILI9468_PASET: u16 = 0x2B;
const ILI9468_RAMWR: u16 = 0x2C;
const ILI9468_RAMRD: u16 = 0x2E;
const ILI9468_MADCTL: u16 = 0x36;


fn start_spi<I>(cs: &mut Pin<I, Output<PushPull>>)
    where
        I: PinId
{
    cs.set_low().unwrap();
}

fn stop_spi<I>(cs: &mut Pin<I, Output<PushPull>>)
    where
        I: PinId
{
    cs.set_high().unwrap();
}

fn spi_write<S, I, Y>(di: & SPIInterface<Spi<Enabled, S, 8>, Pin<I, Output<PushPull>>, Pin<Y, Output<PushPull>>>, bc: u8)
    where
        S: SpiDevice,
        I: PinId,
        Y: PinId,
{
    // spi.write(&[bc]).unwrap();
}

fn command_write<S, I, Y>(di: & SPIInterface<Spi<Enabled, S, 8>, Pin<I, Output<PushPull>>, Pin<Y, Output<PushPull>>>, bc: u8)
where
    S: SpiDevice,
    I: PinId,
    Y: PinId,

{   
    rprintln!("Writing command.");
    // spi.write(&[bc]).unwrap_or_else(|err| { rprintln!("Error!"); });

}


#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Starting program...");
    
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
    let mut _sck_pin = pins.gpio2.into_mode::<gpio::FunctionSpi>();
    let mut _mosi_pin = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let mut cs_pin = pins.gpio5.into_push_pull_output();

    let mut led_pin = pins.led.into_push_pull_output();

    // Data/clock switch pin
    let mut dc_pin = pins.gpio6.into_push_pull_output();

    let mut rst_pin = pins.gpio7.into_push_pull_output();
    rst_pin.set_high().unwrap();


    // SPI configuration
    let mut spi = rp_pico::hal::spi::Spi::<_,_,8>::new(pac.SPI0);
    // 20MHz is max frequency for this screen.
    let mut spi = spi.init(&mut pac.RESETS, clocks.peripheral_clock.freq(), 10.MHz(), &embedded_hal::spi::MODE_3);

    let di = SPIInterface::new(spi, dc_pin, cs_pin);

    // Try to initalize screen here:

    
    // Do a soft reset here:
    command_write(&di, 0x01);
    delay.delay_ms(120);

    // Pixel Format
    command_write(&di, 0x3A);
    spi_write(&di, 0x55);

    // Memory format
    command_write(&di, 0x36);
    spi_write(&di, 0x48);

    // Positive Gamma control (PGAM CTRL)
    command_write(&di, 0xE0);
    spi_write(&di, 0x0F);
    spi_write(&di, 0x1F);
    spi_write(&di, 0x1C);
    spi_write(&di, 0x0C);
    spi_write(&di, 0x0F);
    spi_write(&di, 0x08);
    spi_write(&di, 0x48);
    spi_write(&di, 0x98);
    spi_write(&di, 0x37);
    spi_write(&di, 0x0A);
    spi_write(&di, 0x13);
    spi_write(&di, 0x04);
    spi_write(&di, 0x11);
    spi_write(&di, 0x0D);
    spi_write(&di, 0x00);

    // Negative Gamma control (NGAM CTRL)
    command_write(&di, 0xE1);
    spi_write(&di, 0x0F);
    spi_write(&di, 0x32);
    spi_write(&di, 0x2E);
    spi_write(&di, 0x0B);
    spi_write(&di, 0x0D);
    spi_write(&di, 0x05);
    spi_write(&di, 0x47);
    spi_write(&di, 0x75);
    spi_write(&di, 0x37);
    spi_write(&di, 0x06);
    spi_write(&di, 0x10);
    spi_write(&di, 0x03);
    spi_write(&di, 0x24);
    spi_write(&di, 0x20);
    spi_write(&di, 0x00);


    // Digital Gamma Control (DGAM CTRL)
    command_write(&di, 0xE2);
    spi_write(&di, 0x0F);
    spi_write(&di, 0x32);
    spi_write(&di, 0x2E);
    spi_write(&di, 0x0B);
    spi_write(&di, 0x0D);
    spi_write(&di, 0x05);
    spi_write(&di, 0x47);
    spi_write(&di, 0x75);
    spi_write(&di, 0x37);
    spi_write(&di, 0x06);
    spi_write(&di, 0x10);
    spi_write(&di, 0x03);
    spi_write(&di, 0x24);
    spi_write(&di, 0x20);
    spi_write(&di, 0x00);

    command_write(&di, 0x11);
    delay.delay_ms(150);
    command_write(&di, 0x29);

    delay.delay_ms(150);

    command_write(&di, 0x2A);
    spi_write(&di, 0x00);
    spi_write(&di, 0xFF);
    spi_write(&di, 0x20);
    spi_write(&di, 0xFF);

    command_write(&di, 0x2B);
    spi_write(&di, 0x00);
    spi_write(&di, 0xFF);
    spi_write(&di, 0x20);
    spi_write(&di, 0xFF);

    command_write(&di, 0x2C);

    let mut i = 0;
    while(i<32*32){
        i+=1;
        spi_write(&di, 0x0F);
    }



    loop {
        rprintln!("Running...");
        delay.delay_ms(1000);
    }
}