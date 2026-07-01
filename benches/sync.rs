use criterion::{BenchmarkId, Criterion};
use dynamixel2::client::Client;
use dynamixel2::{bus::Data, client::SyncWriteData};

mod common;

const PRESENT_POSITION: u16 = 132;
const GOAL_POSITION: u16 = 116;

fn sync_read(c: &mut Criterion, client: &mut Client, ids: &[u8], name: &str) {
	let parameter = format!("(num_id:{},baud_rate:{})", ids.len(), client.baud_rate());
	// `sync_read` only reports errors while draining the replies (on drop here), so `.unwrap()` on the
	// write is safe even when an id never responds; the drop just waits out one read timeout.
	c.bench_with_input(BenchmarkId::new(name, parameter), &(), |b, _| {
		b.iter(|| _ = client.sync_read_bytes::<Vec<u8>>(ids, 0, 64).unwrap())
	});
}

fn fast_sync_read(c: &mut Criterion, client: &mut Client, ids: &[u8], name: &str, expect_response: bool) {
	let parameter = format!("(num_id:{},baud_rate:{})", ids.len(), client.baud_rate());
	// `fast_sync_read` reads the whole combined status packet eagerly, so a missing id makes the call
	// itself return a timeout error. Only unwrap when every id is expected to respond.
	c.bench_with_input(BenchmarkId::new(name, parameter), &(), |b, _| {
		b.iter(|| {
			let response = client.fast_sync_read_bytes::<Vec<u8>>(ids, 0, 64);
			if expect_response {
				response.unwrap();
			}
		})
	});
}

fn sync_write(c: &mut Criterion, client: &mut Client, ids: &[u8]) {
	let mut group = c.benchmark_group("sync_write");
	group.sampling_mode(criterion::SamplingMode::Flat);
	let value = 2000;
	let data = ids
		.iter()
		.map(|motor_id| SyncWriteData {
			motor_id: *motor_id,
			data: value,
		})
		.collect::<Vec<_>>();

	let parameter = format!(
		"(num_id:{},len_data:{},baud_rate:{})",
		ids.len(),
		i32::ENCODED_SIZE,
		client.baud_rate()
	);
	group.bench_with_input(BenchmarkId::new("sync_write", parameter), &data, {
		move |b, data| b.iter(|| client.sync_write(GOAL_POSITION, data).unwrap())
	});
}

fn main() {
	let args = common::parse();
	let mut client = common::open_client(&args);
	let mut c = common::criterion(&args);

	sync_read(&mut c, &mut client, &args.ids, "sync_read");
	fast_sync_read(&mut c, &mut client, &args.ids, "fast_sync_read", true);
	sync_write(&mut c, &mut client, &args.ids);

	// Measure the timeout path when one or more ids never reply, saved under separate names so the
	// results stay distinct from the all-responding runs above.
	if !args.unresponsive_ids.is_empty() {
		let ids: Vec<u8> = args.ids.iter().chain(&args.unresponsive_ids).copied().collect();
		sync_read(&mut c, &mut client, &ids, "sync_read_unresponsive");
		fast_sync_read(&mut c, &mut client, &ids, "fast_sync_read_unresponsive", false);
	}

	c.final_summary();
}
