use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion};
use dynamixel2::{bus::Data, client::SyncWriteData};

mod common;

const PRESENT_POSITION: u16 = 132;
const GOAL_POSITION: u16 = 116;

pub fn sync_read(c: &mut Criterion) {
	let (id, mut client) = common::setup();
	let parameter = format!("(num_id:{},baud_rate:{})", id.len(), client.baud_rate());
	c.bench_with_input(BenchmarkId::new("sync_read", parameter), &(), |b, _| {
		b.iter(|| _ = client.sync_read::<u32>(&id, PRESENT_POSITION).unwrap())
	});
}
pub fn sync_write(c: &mut Criterion) {
	let mut c = c.benchmark_group("sync_write");
	c.sampling_mode(criterion::SamplingMode::Flat);
	let (id, mut client) = common::setup();
	let value = 2000;
	let data = id
		.iter()
		.map(|motor_id| SyncWriteData {
			motor_id: *motor_id,
			data: value,
		})
		.collect::<Vec<_>>();

	let parameter = format!(
		"(num_id:{},len_data:{},baud_rate:{})",
		id.len(),
		i32::ENCODED_SIZE,
		client.baud_rate()
	);
	c.bench_with_input(BenchmarkId::new("sync_write", parameter), &data, {
		move |b, data| b.iter(|| client.sync_write(GOAL_POSITION, data).unwrap())
	});
}

criterion_group!(benches, sync_read, sync_write);
criterion_main!(benches);
