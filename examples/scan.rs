use serial::SerialPort;

fn main() {
	if do_main().is_err() {
		std::process::exit(1);
	}
}

fn print_usage() {
	eprintln!("usage: ping TTY BAUD-RATE MOTOR-ID");
}

fn do_main() -> Result<(), ()> {
	let mut args = std::env::args();

	let _ = args.next().unwrap();

	#[cfg(feature = "log")]
	{
		env_logger::from_env("RUST_LOG").filter_level(log::LevelFilter::Trace).init();
	}

	let tty = args.next().ok_or_else(print_usage)?;
	let baud_rate = args.next().ok_or_else(print_usage)?;

	let baud_rate: usize = baud_rate.parse().map_err(|_| eprintln!("invalid baud rate: {}", baud_rate))?;

	let baud_rate = match baud_rate {
		110 => serial::Baud110,
		300 => serial::Baud300,
		600 => serial::Baud600,
		1200 => serial::Baud1200,
		2400 => serial::Baud2400,
		4800 => serial::Baud4800,
		9600 => serial::Baud9600,
		19200 => serial::Baud19200,
		38400 => serial::Baud38400,
		57600 => serial::Baud57600,
		115200 => serial::Baud115200,
		other => serial::BaudOther(other),
	};

	let mut tty = serial::open(&tty).map_err(|e| eprintln!("failed to open serial port at {}: {}", tty, e))?;

	let config = serial::PortSettings {
		baud_rate,
		char_size: serial::Bits8,
		stop_bits: serial::Stop1,
		flow_control: serial::FlowNone,
		parity: serial::ParityNone,
	};

	eprintln!("configuring serial port with: {:#?}", config);
	tty.configure(&config)
		.map_err(|e| eprintln!("failed to configure serial port: {}", e))?;

	dynamixel2::scan(&mut tty, |response| {
		println!("{:#?}", response);
	})
	.map_err(|e| eprintln!("{}", e))?;

	Ok(())
}
