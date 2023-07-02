use cortex_m::delay::Delay;
use embedded_graphics_core::pixelcolor::Rgb565;
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

use embedded_graphics_core::prelude::{Dimensions, Point, Size, IntoStorage};
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::primitives::Rectangle;
use embedded_graphics_core::pixelcolor::{Bgr565, raw::RawU16};
use embedded_graphics_core::Pixel;


use crate::interface::Interface;

const ILI9486_MAD_BGR: u16 = 0x08;
const ILI9486_MAD_MX: u16 = 0x40;
const ILI9486_MAD_MY: u16 = 0x80;
const ILI9486_MAD_MV: u16 = 0x20;
const ILI9486_PAS: u16 = 0x2B;
const ILI9486_CAS: u16 = 0x2A;
const ILI9486_RAMWR: u16 = 0x2C;

pub enum Orientation {
    Landscape,
    Portrait,
    InvertedLandscape,
    InvertedPortrait,
}


pub struct DisplayDriver<D, PID1, PID2, PID3> 
where
    D: SpiDevice,
    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    interface: Interface<D, PID1, PID2, PID3>,
    width: u32,
    height: u32,
    orientation: Orientation,
}

impl<D, PID1, PID2, PID3> Dimensions for DisplayDriver<D, PID1, PID2, PID3>
where
    D: SpiDevice,

    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    fn bounding_box(&self) -> Rectangle {
        return Rectangle { top_left: Point::new(0, 0), size: Size::new(self.width as u32, self.height as u32) }
    }

}

impl<D, PID1, PID2, PID3> DrawTarget for DisplayDriver<D, PID1, PID2, PID3>
where
    D: SpiDevice,

    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;
    
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>
    {
        for Pixel(coord, color) in pixels.into_iter() {
           if coord.x > 0 && coord.x < self.width as i32 && coord.y > 0 && coord.y < self.height as i32 {
                self.draw_rect(coord.x as u32, coord.y as u32, 1, 1, color.into_storage()); // Should double check that this does what i expect. (Perhaps in its own fucntion)
           }
           
        }

        Ok(())
    }

}


impl<D, PID1, PID2, PID3> DisplayDriver<D, PID1, PID2, PID3>
where
    D: SpiDevice,

    PID1: PinId,
    PID2: PinId,
    PID3: PinId,
{
    pub fn new(interface: Interface<D, PID1, PID2, PID3>, width: u32, height: u32, orientation: Orientation) -> DisplayDriver<D, PID1, PID2, PID3>{
        return DisplayDriver { interface: interface, width: width, height: height, orientation: orientation}
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
        // If you have "strobes" on your display, you can try to adjust the value of this. Lower number, means higher current consumption, but better performance.
        self.interface.write_data_iter( &[0x35]);


        // Positive Gamma control (PGAM CTRL)
        let pgam_ctrl: [u8; 16] = [0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98, 0x37, 0x0A, 0x13, 0x04, 0x11, 0x11, 0x0D, 0x00];
        self.interface.write_command( 0xE0);
        self.interface.write_data_iter( &pgam_ctrl);

        // Negative Gamma control (NGAM CTRL)
        self.interface.write_command( 0xE1);
        let ngam_ctrl: [u8; 15] = [0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75, 0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00];
        self.interface.write_data_iter( &ngam_ctrl);

        // Inv off
        self.interface.write_command( 0x20);

        

        // Memory format
        self.interface.write_command( 0x36);
        match self.orientation {
            Orientation::Landscape => {
                self.interface.write_data_iter( &[(ILI9486_MAD_BGR | ILI9486_MAD_MV) as u8]);
            },
            Orientation::Portrait => {
                self.interface.write_data_iter( &[(ILI9486_MAD_BGR | ILI9486_MAD_MX) as u8]);
            },
            Orientation::InvertedLandscape => {
                self.interface.write_data_iter( &[(ILI9486_MAD_BGR | ILI9486_MAD_MV | ILI9486_MAD_MX | ILI9486_MAD_MY) as u8]);
            },
            Orientation::InvertedPortrait => {
                self.interface.write_data_iter( &[(ILI9486_MAD_BGR | ILI9486_MAD_MY) as u8]);
            }
        }
        

        // Display On
        self.interface.write_command( 0x29);

        delay.delay_ms(150);


    }

    pub fn draw_rect(&mut self, x: u32, y: u32, w: u32, h: u32, color: u16){

        self.interface.begin_write();
        self.select_window( x, y, x+w-1, y+h-1,);
        let size: u32 = (w * h) as u32;
        self.write_block(color, size);
        self.interface.end_write();

    }


    pub fn clear_screen(&mut self, color: u16){
        self.interface.begin_write();
        self.select_window( 0, 0, self.width - 1, self.height - 1);
        let size: u32 = self.width * self.height;
        self.write_block(color, size);
        self.interface.end_write();

    }

    pub fn select_window(&mut self, x0: u32, y0: u32, x1: u32, y1: u32){
        // Select window, will always be based on the default orientation of the display.
        // Where the actual datum is / should be, should be handeled by the drawing function.

        let mut dx0 = x0;
        let mut dx1 = x1;
        let mut dy0 = y0;
        let mut dy1 = y1;

        // Check constraints of memory.
        if x0 < 1 {
            dx0 = 0;
        }
        if y0 < 1 {
            dy0 = 0;
        }
        if x1 > self.width {
            dx1 = self.width - 1;
        }
        if y1 > self.height {
            dy1 = self.height - 1;
        }

        self.interface.quick_command(ILI9486_CAS);
        self.interface.transfer(dx0 as u16>>8);
        self.interface.transfer(dx0 as u16);
        self.interface.transfer(dx1 as u16>>8);
        self.interface.transfer(dx1 as u16);

        self.interface.quick_command(ILI9486_PAS);
        self.interface.transfer(dy0 as u16>>8);
        self.interface.transfer(dy0 as u16);
        self.interface.transfer(dy1 as u16>>8);
        self.interface.transfer(dy1 as u16);

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
