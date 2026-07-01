use assert2::let_assert;
use clap::Parser;
use criterion::Criterion;
use dynamixel2::client::Client;
use dynamixel2::serial2::SerialPort;

/// Serial connection settings and benchmark options, taken from the command line.
///
/// The benchmarks talk to real hardware, so the port, baud rate and motor IDs must be given
/// on the command line, e.g. `cargo bench --bench single -- --path /dev/ttyUSB0 --baud 1000000 --ids 1,2`.
#[derive(Parser, Debug)]
#[command(about = "dynamixel2 hardware benchmarks")]
pub struct Args {
	/// Path to the serial port.
	#[arg(long, default_value = "/dev/ttyUSB0")]
	pub path: String,

	/// Baud rate of the serial connection.
	#[arg(long, default_value_t = 56_700)]
	pub baud: u32,

	/// Comma-separated list of motor IDs to address.
	#[arg(long, value_delimiter = ',', default_value = "1,2")]
	pub ids: Vec<u8>,

	/// Comma-separated list of motor IDs that are expected NOT to respond.
	///
	/// When set, extra `sync_read`/`fast_sync_read` benchmarks run against the union of `--ids` and
	/// these IDs, recorded under separate criterion names so the timeout path stays distinct from the
	/// all-responding results.
	#[arg(long, value_delimiter = ',')]
	pub unresponsive_ids: Vec<u8>,

	/// Configure the port for half-duplex RS-485 (requires building with `--features rs4xx`).
	#[arg(long)]
	pub rs485: bool,

	/// Save results under a named criterion baseline.
	#[arg(long)]
	pub save_baseline: Option<String>,

	/// Compare results against a previously-saved criterion baseline.
	#[arg(long, conflicts_with = "save_baseline")]
	pub baseline: Option<String>,

	/// Number of samples criterion collects per benchmark.
	#[arg(long)]
	pub sample_size: Option<usize>,

	/// Only run benchmarks whose names contain this string.
	#[arg(value_name = "FILTER")]
	pub filter: Option<String>,

	/// Absorbs the `--bench` flag that `cargo bench` passes to the harness.
	#[arg(long, hide = true)]
	bench: bool,

	/// Absorbs the `--test` flag that `cargo test` passes to the harness.
	#[arg(long, hide = true)]
	test: bool,
}

/// Parse the benchmark arguments from the command line.
pub fn parse() -> Args {
	Args::parse()
}

/// Open the serial port described by `args` and wrap it in a [`Client`].
pub fn open_client(args: &Args) -> Client {
	let_assert!(
		Ok(port) = SerialPort::open(&args.path, args.baud),
		"unable to open serial port {} at baud {}",
		args.path,
		args.baud
	);
	if args.rs485 {
		set_rs485(&port);
	}
	let write_buffer = vec![0; 256];
	let read_buffer = vec![0; 256];
	let_assert!(
		Ok(client) = Client::with_buffers(port, read_buffer, write_buffer),
		"unable to create client on {}",
		args.path
	);

	println!(
		"client setup, path: {}, baud: {}, ids: {:?}, rs485: {}",
		args.path, args.baud, args.ids, args.rs485
	);
	client
}

#[cfg(feature = "rs4xx")]
fn set_rs485(port: &SerialPort) {
	use dynamixel2::serial2::rs4xx::{Rs485Config, TransceiverMode};
	let_assert!(
		Ok(()) = port.set_rs4xx_mode(TransceiverMode::Rs485(Rs485Config::new())),
		"unable to enable RS-485 mode on the serial port"
	);
}

#[cfg(not(feature = "rs4xx"))]
fn set_rs485(_port: &SerialPort) {
	panic!("--rs485 requires building the benches with `--features rs4xx`");
}

/// Build a [`Criterion`] instance configured from `args`.
///
/// The benchmarks own their `main`, so the usual criterion command-line options are not parsed
/// automatically; the ones that are useful for a hardware bench series are re-exposed on [`Args`].
pub fn criterion(args: &Args) -> Criterion {
	let mut criterion = Criterion::default();
	if let Some(filter) = &args.filter {
		criterion = criterion.with_filter(filter.clone());
	}
	if let Some(sample_size) = args.sample_size {
		criterion = criterion.sample_size(sample_size);
	}
	if let Some(baseline) = &args.save_baseline {
		criterion = criterion.save_baseline(baseline.clone());
	} else if let Some(baseline) = &args.baseline {
		criterion = criterion.retain_baseline(baseline.clone(), true);
	}
	criterion
}
