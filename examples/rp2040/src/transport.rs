use adafruit_kb2040::hal;
use adafruit_kb2040::hal::fugit::ExtU64;
use adafruit_kb2040::hal::timer::Instant;
use adafruit_kb2040::hal::uart::{Enabled, UartDevice, UartPeripheral, ValidUartPinout};
use adafruit_kb2040::hal::Timer;
use core::convert::Infallible;
use core::time::Duration;
use cortex_m::prelude::_embedded_hal_serial_Write;
use dynamixel2::SerialPort;
use embedded_hal::digital::OutputPin;
use nb::block;

use hal::uart::ReadErrorType;

#[derive(Debug)]
pub enum Error {
	UartReadError(ReadErrorType),
	Timeout,
}
pub struct DynamixelSerial<P: ValidUartPinout<D>, D: UartDevice, DIR> {
	serial: UartPeripheral<Enabled, D, P>,
	baud_rate: u32,
	dir_pin: DIR,
	timer: Timer,
}

impl<P, D, DIR> DynamixelSerial<P, D, DIR>
where
	P: ValidUartPinout<D>,
	D: UartDevice,
{
	pub fn new(serial: UartPeripheral<Enabled, D, P>, baud_rate: u32, dir_pin: DIR, timer: Timer) -> Self {
		Self {
			serial,
			baud_rate,
			dir_pin,
			timer,
		}
	}
}

impl<P, D, DIR> SerialPort for DynamixelSerial<P, D, DIR>
where
	P: ValidUartPinout<D>,
	D: UartDevice,
	DIR: OutputPin<Error = Infallible>,
{
	type Error = Error;

	type Instant = Instant;

	fn baud_rate(&self) -> Result<u32, Self::Error> {
		Ok(self.baud_rate)
	}

	fn set_baud_rate(&mut self, _baud_rate: u32) -> Result<(), Self::Error> {
		panic!("Changing baud rate is not supported");
	}

	fn discard_input_buffer(&mut self) -> Result<(), Self::Error> {
		Ok(())
	}

	fn read(&mut self, buffer: &mut [u8], deadline: &Self::Instant) -> Result<usize, Self::Error> {
		loop {
			if deadline < &self.timer.get_counter() {
				return Err(Error::Timeout);
			}
			match self.serial.read_raw(buffer) {
				Err(nb::Error::Other(e)) => return Err(Error::UartReadError(e.err_type)),
				Err(nb::Error::WouldBlock) => {},
				Ok(x) => return Ok(x),
			}
		}
	}

	fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
		let _ = self.dir_pin.set_high();
		self.serial.write_full_blocking(buffer);
		let _ = block!(self.serial.flush());
		let _ = self.dir_pin.set_low();
		Ok(())
	}

	fn make_deadline(&self, timeout: Duration) -> Self::Instant {
		let timeout = timeout.as_millis() as u64;
		self.timer.get_counter() + timeout.millis()
	}

	fn is_timeout_error(error: &Self::Error) -> bool {
		matches!(error, Error::Timeout)
	}
}
