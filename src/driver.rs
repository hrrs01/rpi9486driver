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

use embedded_graphics_core::prelude::{Dimensions, Point, Size};
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::pixelcolor::Bgr565;
use embedded_graphics_core::Pixel;


use crate::interface::Interface;

const ILI9486_MAD_BGR: u16 = 0x08;
const ILI9486_MAD_MX: u16 = 0x40;
const ILI9486_PAS: u16 = 0x2B;
const ILI9486_CAS: u16 = 0x2A;
const ILI9486_RAMWR: u16 = 0x2C;

pub struct DisplayDriver<D, PID1, PID2, PID3> 
where
    D: SpiDevice,
    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    interface: Interface<D, PID1, PID2, PID3>
}

impl<D, PID1, PID2, PID3> Dimensions for DisplayDriver<D, PID1, PID2, PID3>
where
    D: SpiDevice,

    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    fn bounding_box(&self) -> Rectangle {
        return Rectangle { top_left: Point::new(0, 0), size: Size::new(480, 320) }
    }

}

impl<D, PID1, PID2, PID3> DrawTarget for DisplayDriver<D, PID1, PID2, PID3>
where
    D: SpiDevice,

    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    type Color = Bgr565;
    type Error = core::convert::Infallible;
    
    fn draw_iter<I>(&mut self, drawables: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>
    {
        todo!();
    }

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
        self.select_window( x, y, x+w-1, y+h-1);
        let size: u32 = (w*h) as u32;
        self.write_block(color, size);
        self.interface.end_write();

    }


    pub fn clear_screen(&mut self, color: u16){
        self.interface.begin_write();
        self.select_window( 0, 0, 319, 479);
        let size: u32 = 480 * 320;
        self.write_block(color, size);
        self.interface.end_write();

    }

    pub fn select_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16){

        self.interface.quick_command(ILI9486_CAS);
        self.interface.transfer(x0>>8);
        self.interface.transfer(x0);
        self.interface.transfer(x1>>8);
        self.interface.transfer(x1);

        self.interface.quick_command(ILI9486_PAS);
        self.interface.transfer(y0>>8);
        self.interface.transfer(y0);
        self.interface.transfer(y1>>8);
        self.interface.transfer(y1);

        self.interface.quick_command(ILI9486_RAMWR);
        

    }

    pub fn write_block(&mut self, color: u16, size: u32){
        let mut remainder = size.clone();
        while remainder > 0 {
            self.interface.transfer(color);
            remainder -= 1;
        }
        
    }

}
