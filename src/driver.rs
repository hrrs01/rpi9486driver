use cortex_m::delay::Delay;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::spi::{Mode, MODE_0};
use embedded_hal::{digital::v2::OutputPin, prelude::{_embedded_hal_blocking_spi_Write, _embedded_hal_blocking_delay_DelayUs}};
use fugit::HertzU32;
use rp2040_pac::RESETS;
use rp_pico::hal::spi::Disabled;
use rp_pico::pac::generic::Reg;
use rp_pico::pac::spi1::sspcr0::SSPCR0_SPEC;
use rp_pico::{hal::{Spi, spi::{Enabled, SpiDevice}, gpio::{Pin, PinId, Output, PushPull}}};
use fugit::RateExtU32;

const ILI9486_MAD_BGR: u16 = 0x08;
const ILI9486_MAD_MX: u16 = 0x40;
const ILI9486_PAS: u16 = 0x2B;
const ILI9486_CAS: u16 = 0x2A;
const ILI9486_RAMWR: u16 = 0x2C;


pub struct Interface<D, PID1, PID2, PID3>
where
    D: SpiDevice,
    PID1: PinId,
    PID2: PinId,
    PID3: PinId,

{
    pub spi: Spi<Enabled, D, 16>,
    pub cs_pin: Pin<PID1, Output<PushPull>>,
    pub dc_pin: Pin<PID2, Output<PushPull>>,
    pub rst_pin: Pin<PID3, Output<PushPull>>,

}

impl<D, PID1, PID2, PID3> Interface<D, PID1, PID2, PID3>
where
    D: SpiDevice,
    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    fn begin_write(&mut self){
        self.cs_pin.set_low().unwrap();
    }

    fn send_command(&mut self, byte: u8){
        self.dc_pin.set_low().unwrap();
        self.transfer8(byte);
        self.dc_pin.set_high().unwrap();
    }

    fn quick_commmand(&mut self, word: u16){
        self.wait_til_clear();
        self.dc_pin.set_low().unwrap();
        self.transfer(word);
        self.dc_pin.set_high().unwrap();
    }

    fn transfer(&mut self, word: u16){
        let bytes = word.to_be_bytes();
        self.spi.write(&[word]).unwrap();
    }



    fn end_write(&mut self){
        self.cs_pin.set_high().unwrap();
    }

    fn transfer8(&mut self, byte: u8){
        let mut word: u16 = (byte as u16) << 8 | (byte as u16);
        self.wait_til_clear();
        self.spi.write(&[word]).unwrap();
    }

    pub fn write_command(&mut self, byte: u8){
        self.wait_til_clear();
        self.begin_write();
        self.send_command(byte);
        self.end_write();
    }

    pub fn write_data(&mut self, byte: u8){
        self.wait_til_clear();
        self.begin_write();
        self.transfer8(byte);
        self.end_write();
    }

    pub fn write_data_iter(&mut self, bytes: &[u8]){
        for byte in bytes.iter() {
            self.write_data(byte.clone());
        }
    }

    pub fn hard_reset(&mut self, delay: &mut Delay){
        self.rst_pin.set_low().unwrap();
        delay.delay_ms(2);
        self.rst_pin.set_high().unwrap();

    }

    fn select_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16){

        self.quick_commmand(ILI9486_CAS);
        self.transfer(x0>>8);
        self.transfer(x0);
        self.transfer(x1>>8);
        self.transfer(x1);

        self.quick_commmand(ILI9486_PAS);
        self.transfer(y0>>8);
        self.transfer(y0);
        self.transfer(y1>>8);
        self.transfer(y1);

        self.quick_commmand(ILI9486_RAMWR);
        

    }

    fn write_block(&mut self, color: u16, size: u32){
        let mut remainder = size.clone();
        while remainder > 0 {
            self.transfer(color);
            remainder -= 1;
        }
        
    }



    fn wait_til_clear(&mut self){
        while(self.spi.is_busy()){}
    }
}


pub struct DisplayDriver<D, PID1, PID2, PID3> 
where
    D: SpiDevice,
    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    interface: Interface<D, PID1, PID2, PID3>
}

impl<D, PID1, PID2, PID3> DisplayDriver<D, PID1, PID2, PID3>
where
    D: SpiDevice,

    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    pub fn new(interface: Interface<D, PID1, PID2, PID3>) -> DisplayDriver<D, PID1, PID2, PID3>{
        return DisplayDriver { interface: interface}
    }

    pub fn init(&mut self, delay: &mut Delay){
        // Do a soft reset here:
        self.interface.hard_reset(delay);

        self.interface.write_command( 0x01);
        delay.delay_ms(120);

        self.interface.write_command( 0x11);
        delay.delay_ms(120);

        // Pixel Format
        self.interface.write_command( 0x3A);
        self.interface.write_data_iter( &[0x55]);

        // Power Control 1
        self.interface.write_command( 0xC0);
        self.interface.write_data_iter( &[0x0E, 0x0E]);

        // Power Control 2
        self.interface.write_command( 0xC1);
        self.interface.write_data_iter( &[0x41, 0x00]);

        // Power Control 3
        self.interface.write_command( 0xC2);
        self.interface.write_data_iter( &[0x55]);


        // Positive Gamma control (PGAM CTRL)
        let pgam_ctrl: [u8; 16] = [0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x11, 0x0D, 0x00];
        self.interface.write_command( 0xE0);
        self.interface.write_data_iter( &pgam_ctrl);

        // Negative Gamma control (NGAM CTRL)
        self.interface.write_command( 0xE1);
        let ngam_ctrl: [u8; 15] = [0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00];
        self.interface.write_data_iter( &ngam_ctrl);

        // Digital Gamma Control (DGAM CTRL)
        self.interface.write_command( 0xE2);
        self.interface.write_data_iter( &ngam_ctrl);

        // Inv off
        self.interface.write_command( 0x20);

        // Memory format
        self.interface.write_command( 0x36);   
        self.interface.write_data_iter( &[(ILI9486_MAD_BGR | ILI9486_MAD_MX) as u8]);

        // Display On
        self.interface.write_command( 0x29);

        delay.delay_ms(150);


    }

    pub fn draw_rect(&mut self, x: u16, y: u16, w: u16, h: u16, color: u16){
        self.interface.begin_write();
        self.interface.select_window( x, y, x+w-1, y+h-1);
        let size: u32 = (w*h) as u32;
        self.interface.write_block(color, size);
        self.interface.end_write();

    }


    pub fn clear_screen(&mut self, color: u16){
        self.interface.begin_write();
        self.interface.select_window( 0, 0, 319, 479);
        let size: u32 = 480 * 320;
        self.interface.write_block(color, size);
        self.interface.end_write();

    }   
}
