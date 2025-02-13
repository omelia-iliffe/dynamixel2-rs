//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod transport;

use bsp::entry;
use core::convert::Infallible;
use core::time::Duration;
use defmt::{todo, unreachable, *};
use defmt_rtt as _;
use panic_probe as _;

use crate::transport::DynamixelSerial;
use adafruit_kb2040 as bsp;
use adafruit_kb2040::hal;
use adafruit_kb2040::hal::fugit::RateExtU32;
use adafruit_kb2040::hal::uart::{DataBits, StopBits, UartConfig, UartDevice, ValidUartPinout};
use bsp::hal::{
	clocks::{init_clocks_and_plls, Clock},
	pac,
	sio::Sio,
	watchdog::Watchdog,
};
use dynamixel2::{Bus, Device, Instructions, ReadError, SerialPort, TransferError};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
const ID: u8 = 1;
const LED_ADDRESS: u16 = 65;
const BAUD_RATE: u32 = 57600;

#[cfg(all(feature = "client", feature = "device"))]
compile_error!("cannot have both client and device features enabled");
#[cfg(all(not(feature = "client"), not(feature = "device")))]
compile_error!("must have either client and device feature enabled");

#[entry]
fn main() -> ! {
	info!("Program start");
	let mut pac = pac::Peripherals::take().unwrap();
	let _core = pac::CorePeripherals::take().unwrap();
	let mut watchdog = Watchdog::new(pac.WATCHDOG);
	let sio = Sio::new(pac.SIO);

	// External high-speed crystal on the pico board is 12Mhz
	let external_xtal_freq_hz = 12_000_000u32;
	let clocks = init_clocks_and_plls(
		external_xtal_freq_hz,
		pac.XOSC,
		pac.CLOCKS,
		pac.PLL_SYS,
		pac.PLL_USB,
		&mut pac.RESETS,
		&mut watchdog,
	)
	.ok()
	.unwrap();

	let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

	let pins = bsp::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
	// SETUP THE UART PINS
	let uart_pins = (
		// UART TX
		pins.d4.into_function(),
		// UART RX
		pins.d5.into_function(),
	);
	// CREATE THE UART PERIPHERAL
	let uart = hal::uart::UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
		.enable(
			UartConfig::new(BAUD_RATE.Hz(), DataBits::Eight, None, StopBits::One),
			clocks.peripheral_clock.freq(),
		)
		.unwrap();
	// SETUP THE ENABLE/DIR PIN
	let dir_pin = pins.d6.into_function();
	// CREATE THE TRANSPORT
	let transport = DynamixelSerial::new(uart, BAUD_RATE, dir_pin, timer);

	if cfg!(all(feature = "client", not(feature = "device"))) {
		client_loop(transport, timer)
	} else if cfg!(all(feature = "device", not(feature = "client"))) {
		device_loop(transport)
	} else {
		unreachable!("either client or device feature must be enabled")
	}
}

fn client_loop<P, D, DIR>(transport: DynamixelSerial<P, D, DIR>, mut timer: hal::Timer) -> !
where
	P: ValidUartPinout<D>,
	D: UartDevice,
	DIR: OutputPin<Error = Infallible>,
{
	// CREATE THE CLIENT
	let mut client = Bus::with_buffers(transport, [0; 200], [0; 200]).unwrap();
	loop {
		info!("led on");
		if let Err(e) = client.write_u8(ID, LED_ADDRESS, 1) {
			error!("Error: {:?}", defmt::Debug2Format(&e));
		}
		timer.delay_ms(500);
		info!("led off");
		if let Err(e) = client.write_u8(ID, LED_ADDRESS, 0) {
			error!("Error: {:?}", Debug2Format(&e));
		}
		timer.delay_ms(500);
	}
}

fn device_loop<P, D, DIR>(transport: DynamixelSerial<P, D, DIR>) -> !
where
	P: ValidUartPinout<D>,
	D: UartDevice,
	DIR: OutputPin<Error = Infallible>,
{
	let mut device = Device::with_buffers(transport, [0; 200], [0; 200]).unwrap();
	loop {
		if let Err(e) = process_packet(&mut device) {
			error!("{:?}", Debug2Format(&e))
		}
	}
}

fn process_packet<ReadBuffer, WriteBuffer, P, D, DIR>(
	device: &mut Device<ReadBuffer, WriteBuffer, DynamixelSerial<P, D, DIR>>,
) -> Result<(), TransferError<transport::Error>>
where
	P: ValidUartPinout<D>,
	D: UartDevice,
	DIR: OutputPin<Error = Infallible>,
	WriteBuffer: AsRef<[u8]> + AsMut<[u8]>,
	ReadBuffer: AsRef<[u8]> + AsMut<[u8]>,
{
	let packet = device.read(Duration::from_millis(10));
	if matches!(&packet,
		Err(ReadError::Io(e)) if DynamixelSerial::<P, D, DIR>::is_timeout_error(&e))
	{
		return Ok(());
	}
	let packet = packet?;
	let id = packet.id;
	if id != ID && id != 254 {
		return Ok(());
	}
	match packet.instruction {
		Instructions::Ping => {
			// todo: this should wait for based on id for some amount of time
			device.write_status(ID, 0, 3, |buffer| {
				buffer[..2].copy_from_slice(&1020_u16.to_le_bytes()); // u16 MODEL NUMBER
				buffer[2] = 1; //u8 FIRMWARE VERSION
			})?;
		},
		Instructions::Read { address, length } => {
			if let Some(data) = todo!("get your data for reading") {
				device.write_status(ID, 0, length as usize, |buffer| {
					buffer.copy_from_slice(data);
				})?;
			} else {
				device.write_status_error(ID, 0x07)?;
			}
		},
		Instructions::Write { address, parameters } => {
			if todo!("perform the write") {
				device.write_status_ok(ID)?;
			} else {
				device.write_status_error(ID, 0x07)?;
			}
		},
		Instructions::Unknown { instruction, .. } => error!("Unknown instruction {:?}", instruction),
		i => {
			warn!("unimplemented instruction: {}", Debug2Format(&i))
		},
	};
	Ok(())
}
