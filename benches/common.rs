use assert2::let_assert;
use dynamixel2::client::Client;

const DEFAULT_SERIAL_PATH: &str = "/dev/ttyUSB0";
const DEFAULT_BAUD: u32 = 56_700;
const DEFAULT_IDS: &[u8] = &[1, 2];

pub fn setup() -> (Vec<u8>, Client) {
	let path = std::env::var("SERIAL_PATH").unwrap_or(DEFAULT_SERIAL_PATH.to_string());
	let baud = std::env::var("SERIAL_BAUD")
		.map(|s| {
			let_assert!(Ok(s) = s.parse(), "unable to parse SERIAL_BAUD {} into u32", s);
			s
		})
		.unwrap_or(DEFAULT_BAUD);
	let ids = std::env::var("DEVICE_IDS")
		.map(|ids| {
			ids.split(",")
				.map(|id| {
					let_assert!(Ok(id) = id.parse(), "unable to parse DEVICE_IDS {} into Vec<u8>", ids);
					id
				})
				.collect()
		})
		.unwrap_or(DEFAULT_IDS.to_vec());
	let_assert!(
		Ok(client) = Client::open(&path, baud),
		"unable to open serial port at {} with baud {}",
		path,
		baud
	);

	println!("client setup, baud {baud}, ids: {ids:?}");
	(ids, client)
}
