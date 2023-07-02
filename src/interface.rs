
use cortex_m::delay::Delay;

use embedded_hal::{digital::v2::OutputPin, prelude::{_embedded_hal_blocking_spi_Write,}};

use rp_pico::{hal::{Spi, spi::{Enabled, SpiDevice}, gpio::{Pin, PinId, Output, PushPull}}};




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
    pub fn begin_write(&mut self){
        self.cs_pin.set_low().unwrap();
    }

    pub fn send_command(&mut self, byte: u8){
        self.wait_til_clear();
        self.dc_pin.set_low().unwrap();
        self.transfer8(byte);
        self.dc_pin.set_high().unwrap();
    }

    pub fn quick_command(&mut self, word: u16){
        self.wait_til_clear();
        self.dc_pin.set_low().unwrap();
        self.transfer(word);
        self.dc_pin.set_high().unwrap();
    }

    pub fn transfer(&mut self, word: u16){
        self.spi.write(&[word]).unwrap();
    }



    pub fn end_write(&mut self){
        self.cs_pin.set_high().unwrap();
    }

    pub fn transfer8(&mut self, byte: u8){
        let word: u16 = (byte as u16) << 8 | (byte as u16);
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

    



    fn wait_til_clear(&mut self){
        while self.spi.is_busy(){}
    }
}