use core::convert::Infallible;
use core::time::Duration;
use adafruit_kb2040::hal;
use adafruit_kb2040::hal::fugit::ExtU32;
use adafruit_kb2040::hal::timer::{Alarm, ScheduleAlarmError};
use adafruit_kb2040::hal::uart::{Enabled, UartDevice, UartPeripheral, ValidUartPinout};
use cortex_m::prelude::_embedded_hal_serial_Write;
use embedded_hal::digital::OutputPin;
use dynamixel2::{InitializeError, ReadError, Transport};
use nb::block;

use hal::uart::ReadErrorType as UartReadError;
#[derive(Debug)]
pub enum Error {
    UartReadError(UartReadError),
    ScheduleAlarmError(ScheduleAlarmError)

}
pub struct DynamixelSerial<P: ValidUartPinout<D>, D: UartDevice, A, DIR> {
    serial: UartPeripheral<Enabled, D, P>,
    baud_rate: u32,
    dir_pin: DIR,
    timer: A,
}

impl<P, D, A, DIR> DynamixelSerial<P, D, A, DIR> where P: ValidUartPinout<D>, D: UartDevice, A: Alarm{
    pub fn new(serial: UartPeripheral<Enabled, D, P>, baud_rate: u32, dir_pin: DIR, alarm: A) -> Self {
        Self {
            serial,
            baud_rate,
            dir_pin,
            timer: alarm,
        }
    }
}

impl<P, D, A, DIR> Transport for DynamixelSerial<P, D, A, DIR>
    where P: ValidUartPinout<D>,
    D: UartDevice,
    A: Alarm,
    DIR: OutputPin<Error = Infallible>
{
    type Error = Error;

    fn baud_rate(&self) -> Result<u32, InitializeError<Self::Error>> {
        Ok(self.baud_rate)
    }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> Result<(), Self::Error> {
        panic!("Changing baud rate is not supported");
    }

    fn discard_input_buffer(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_timeout(&mut self, timeout: Duration) -> Result<(), Self::Error> {
        self.timer.schedule((timeout.as_micros() as u32).micros()).map_err(Error::ScheduleAlarmError)?;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, ReadError<Self::Error>> {
        let r = block!(self.serial.read_raw(buffer));
        let r = r.map_err(|e| ReadError::Io(Error::UartReadError(e.err_type)))?;
        Ok(r)
    }

    fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        let _ = self.dir_pin.set_high();
        self.serial.write_full_blocking(buffer);
        block!(self.serial.flush());
        let _ = self.dir_pin.set_low();
        // self.serial.flush().unwrap();
        Ok(())
    }
}