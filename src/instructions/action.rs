use super::{instruction_id, packet_id, Instruction};

#[derive(Debug, Clone)]
pub struct Action {
	pub motor_id: u8,
}

impl Action {
	pub fn unicast(motor_id: u8) -> Self {
		Self { motor_id }
	}

	pub fn broadcast() -> Self {
		Self {
			motor_id: packet_id::BROADCAST,
		}
	}
}

impl Instruction for Action {
	type Response = ();

	fn request_packet_id(&self) -> u8 {
		self.motor_id
	}

	fn request_instruction_id(&self) -> u8 {
		instruction_id::ACTION
	}

	fn request_parameters_len(&self) -> u16 {
		0
	}

	fn encode_request_parameters(&self, _buffer: &mut [u8]) {
		// Empty parameters.
	}

	fn decode_response_parameters(&mut self, packet_id: u8, parameters: &[u8]) -> Result<Self::Response, crate::InvalidMessage> {
		crate::InvalidPacketId::check_ignore_broadcast(packet_id, self.motor_id)?;
		crate::InvalidParameterCount::check(parameters.len(), 0)?;

		Ok(())
	}
}