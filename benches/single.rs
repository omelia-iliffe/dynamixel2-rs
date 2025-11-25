use assert2::let_assert;
use criterion::{criterion_group, criterion_main, Criterion};
use dynamixel2::client::Client;

mod common;

const PRESENT_POSITION: u16 = 132;
const GOAL_POSITION: u16 = 116;

pub fn read(c: &mut Criterion) {
	let (id, mut client) = common::setup();
	c.bench_function("read", |b| b.iter(|| _ = client.read::<u32>(id[0], PRESENT_POSITION).unwrap()));
}
pub fn write(c: &mut Criterion) {
	let (id, mut client) = common::setup();
	c.bench_function("write", |b| b.iter(|| _ = client.write::<u32>(id[0], GOAL_POSITION, &116).unwrap()));
}

criterion_group!(benches, read, write);
criterion_main!(benches);
