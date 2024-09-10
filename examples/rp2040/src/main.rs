//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod transport;

use core::iter::once;
use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use adafruit_kb2040 as bsp;
use adafruit_kb2040::hal;
use adafruit_kb2040::hal::fugit::RateExtU32;
use adafruit_kb2040::hal::uart::{DataBits, Pins, StopBits, UartConfig};
use bsp::hal::{
	clocks::{init_clocks_and_plls, Clock},
	pac,
	sio::Sio,
	watchdog::Watchdog,
};
use embedded_hal::delay::DelayNs;
use dynamixel2::Bus;
use crate::transport::DynamixelSerial;
const ID: u8 = 1;
const LED_ADDRESS: u16 = 65;
const BAUD_RATE: u32 = 57600;
#[entry]
fn main() -> ! {
	info!("Program start");
	let mut pac = pac::Peripherals::take().unwrap();
	let core = pac::CorePeripherals::take().unwrap();
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


	let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
	let alarm = timer.alarm_0().unwrap();

	let pins = bsp::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);
	// SETUP THE UART PINS
	let uart_pins = (
		// UART TX
		pins.d4.into_function(),
		// UART RX
		pins.d5.into_function(),
	);
	/// CREATE THE UART PERIPHERAL
	let uart = hal::uart::UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
		.enable(
			UartConfig::new(BAUD_RATE.Hz(), DataBits::Eight, None, StopBits::One),
			clocks.peripheral_clock.freq(),
		)
		.unwrap();
	/// SETUP THE ENABLE/DIR PIN
	let dir_pin = pins.d6.into_function();
	/// CREATE THE TRANSPORT
	let transport = DynamixelSerial::new(uart, BAUD_RATE, dir_pin, alarm);
	/// CREATE THE BUS
	let mut bus = Bus::with_buffers(transport, [0; 200], [0; 200]).unwrap();
	loop {
		info!("led on");
		bus.write_u8(ID, LED_ADDRESS, 1).unwrap();
		timer.delay_ms(500);
		info!("led off");
		bus.write_u8(ID, LED_ADDRESS, 0).unwrap();
		timer.delay_ms(500);
	}
}
