use criterion::Criterion;
use dynamixel2::client::Client;

mod common;

const PRESENT_POSITION: u16 = 132;
const GOAL_POSITION: u16 = 116;

fn read(c: &mut Criterion, client: &mut Client, ids: &[u8]) {
	c.bench_function("read", |b| b.iter(|| _ = client.read::<u32>(ids[0], PRESENT_POSITION).unwrap()));
}

fn write(c: &mut Criterion, client: &mut Client, ids: &[u8]) {
	c.bench_function("write", |b| {
		b.iter(|| _ = client.write::<u32>(ids[0], GOAL_POSITION, &116).unwrap())
	});
}

fn main() {
	let args = common::parse();
	let mut client = common::open_client(&args);
	let mut c = common::criterion(&args);

	read(&mut c, &mut client, &args.ids);
	write(&mut c, &mut client, &args.ids);

	c.final_summary();
}
