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

use display_interface::WriteOnlyDataCommand;
use display_interface::DataFormat;
use display_interface_spi::SPIInterface;


use fugit::RateExtU32;


const ILI9468_CASET: u8 = 0x2A;
const ILI9468_PASET: u8 = 0x2B;
const ILI9468_RAMWR: u8 = 0x2C;
const ILI9468_RAMRD: u8 = 0x2E;
const ILI9468_MADCTL: u8 = 0x36;
const ILI9468_MAD_BGR: u8 = 0x08;
const ILI9468_MAD_MX: u8 = 0x40;



fn spi_write(di: &mut impl WriteOnlyDataCommand, bc: &[u16])
{

    di.send_data(DataFormat::U16(bc)).unwrap();

}

fn command_write(di: &mut impl WriteOnlyDataCommand , bc: u8)
{   
    rprintln!("Writing command.");
    // spi.write(&[bc]).unwrap_or_else(|err| { rprintln!("Error!"); });
    di.send_commands(DataFormat::U8(&[bc])).unwrap();

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

    let mut di = SPIInterface::new(spi, dc_pin, cs_pin);

    // Try to initalize screen here:

    
    // Do a soft reset here:
    command_write(&mut di, 0x01);
    delay.delay_ms(120);

    command_write(&mut di, 0x11);
    delay.delay_ms(120);

    // Pixel Format
    command_write(&mut di, 0x3A);
    spi_write(&mut di, &[0x55]);

    // Power Control 1
    command_write(&mut di, 0xC0);
    spi_write(&mut di, &[0x0E, 0x0E]);

    // Power Control 2
    command_write(&mut di, 0xC1);
    spi_write(&mut di, &[0x41, 0x00]);

    // Power Control 3
    command_write(&mut di, 0xC2);
    spi_write(&mut di, &[0x55]);


    // Positive Gamma control (PGAM CTRL)
    let pgam_ctrl: [u16; 16] = [0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x11, 0x0D, 0x00];
    command_write(&mut di, 0xE0);
    spi_write(&mut di, &pgam_ctrl);

    // Negative Gamma control (NGAM CTRL)
    command_write(&mut di, 0xE1);
    let ngam_ctrl: [u16; 15] = [0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00];
    spi_write(&mut di, &ngam_ctrl);

    // Digital Gamma Control (DGAM CTRL)
    command_write(&mut di, 0xE2);
    spi_write(&mut di, &ngam_ctrl);

    // Inv off
    command_write(&mut di, 0x20);

    // Memory format
    command_write(&mut di, 0x36);   
    spi_write(&mut di, &[0x48]);

    command_write(&mut di, 0x29);

    delay.delay_ms(150);

    command_write(&mut di, ILI9468_MAD_BGR | ILI9468_MAD_MX);

    let start_x = 0u16;
    let end_x = 200u16;

    command_write(&mut di, 0x2A);
    let mut pager: [u16; 4] = [0x00, start_x, 0x00, end_x];
    spi_write(&mut di, &pager);

    let start_y = 0u16;
    let end_y = 200u16;

    let mut column: [u16; 4] = [0x00; 4];
    column[0] = 0x00;
    column[1] = 0x00;

    column[2] = 0x00;
    column[3] = end_y;

    // column[0..2].copy_from_slice(&start_y.to_be_bytes());
    // column[2..4].copy_from_slice(&end_y.to_be_bytes());


    command_write(&mut di, 0x2B);
    spi_write(&mut di, &column);

    command_write(&mut di, 0x2C);
    let graphics: [u16; 200*200] = [0x0F; 200*200];
    spi_write(&mut di, &graphics);


    loop {
        rprintln!("Running...");
        delay.delay_ms(1000);
    }
}