/// Implementation of the DAP_SWD_SEQUENCE command
use super::super::{CmsisDapError, CommandId, Request, SendError, Status};

use bitvec::prelude::*;

/// One run of SWCLK cycles, with SWDIO either driven by the probe or released.
///
/// Releasing the pin is what the command exists for: multidrop target selection needs the
/// probe to stop driving SWDIO while the clock keeps running.
#[derive(Clone, Debug)]
pub struct Sequence {
    /// Number of SWCLK cycles: 1..64 (64 encoded as 0)
    clock_cycles: u8,

    /// Bits to drive on SWDIO, or `None` to release the pin and capture it instead.
    data: Option<[u8; 8]>,
}

impl Sequence {
    /// Drive `swdio` on SWDIO, one bit per SWCLK cycle, least significant bit first.
    pub(crate) fn output(swdio: &BitSlice) -> Result<Self, CmsisDapError> {
        Self::new(swdio.len(), Some(swdio.load_le::<u64>().to_le_bytes()))
    }

    /// Release SWDIO and capture `clock_cycles` bits from it.
    pub(crate) fn input(clock_cycles: usize) -> Result<Self, CmsisDapError> {
        Self::new(clock_cycles, None)
    }

    fn new(clock_cycles: usize, data: Option<[u8; 8]>) -> Result<Self, CmsisDapError> {
        assert!(
            clock_cycles > 0 && clock_cycles <= 64,
            "clock_cycles = {clock_cycles}, but expected [1,64]"
        );

        Ok(Self {
            clock_cycles: clock_cycles as u8,
            data,
        })
    }

    /// Whether the probe captures this sequence rather than driving it.
    fn captures(&self) -> bool {
        self.data.is_none()
    }

    fn byte_count(&self) -> usize {
        (self.clock_cycles as usize).div_ceil(8)
    }
}

#[derive(Clone, Debug)]
pub struct SequenceRequest {
    sequences: Vec<Sequence>,
}

impl SequenceRequest {
    pub(crate) fn new(sequences: Vec<Sequence>) -> Result<Self, CmsisDapError> {
        assert!(
            !sequences.is_empty() && sequences.len() <= (u8::MAX as usize),
            "sequences.len() == {}, but expected [1,255]",
            sequences.len()
        );
        Ok(SequenceRequest { sequences })
    }
}

impl Request for SequenceRequest {
    const COMMAND_ID: CommandId = CommandId::SwdSequence;

    type Response = SequenceResponse;

    /*
    | BYTE | BYTE **********| BYTE *********| BYTE ******|
    > 0x1D | Sequence Count | Sequence Info | SWDIO Data |
    |******|****************|///////////////|++++++++++++|
     */
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        let mut transfer_len_bytes = 0;
        buffer[transfer_len_bytes] = self.sequences.len() as u8;
        transfer_len_bytes += 1;

        for sequence in self.sequences.iter() {
            // The cycle count has been checked by Sequence::new.
            let clock_cycles = sequence.clock_cycles;

            let mut sequence_info = if clock_cycles == 64 { 0 } else { clock_cycles };
            sequence_info |= (sequence.captures() as u8) << 7;
            buffer[transfer_len_bytes] = sequence_info;
            transfer_len_bytes += 1;

            // An input sequence carries its data in the response instead.
            if let Some(data) = sequence.data {
                let byte_count = sequence.byte_count();
                buffer[transfer_len_bytes..][..byte_count].copy_from_slice(&data[..byte_count]);
                transfer_len_bytes += byte_count;
            }
        }

        Ok(transfer_len_bytes)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        let mut received_len_bytes = 1;
        let status = Status::from_byte(buffer[0])?;

        let mut bits = BitVec::new();

        for sequence in self.sequences.iter().filter(|s| s.captures()) {
            let clock_cycles = sequence.clock_cycles as usize;
            let byte_count = sequence.byte_count();

            if buffer.len() < received_len_bytes + byte_count {
                return Err(SendError::NotEnoughData);
            }

            let bytes = &buffer[received_len_bytes..][..byte_count];
            bits.extend_from_bitslice(&bytes.view_bits::<Lsb0>()[..clock_cycles]);
            received_len_bytes += byte_count;
        }

        Ok(SequenceResponse(status, bits))
    }
}

#[derive(Debug)]
pub struct SequenceResponse(pub(crate) Status, pub(crate) BitVec);

#[cfg(test)]
mod tests {
    use super::*;

    /// The layout a probe sees: a count, then one info byte per sequence, with data only for
    /// the sequences the host drives.
    #[test]
    fn an_output_then_an_input_sequence_is_encoded() {
        let request = SequenceRequest::new(vec![
            Sequence::output(bits![1, 0]).unwrap(),
            Sequence::input(1).unwrap(),
        ])
        .unwrap();

        let mut buffer = [0u8; 16];
        let written = request.to_bytes(&mut buffer).unwrap();

        assert_eq!(&buffer[..written], &[2, 2, 0b01, 0x80 | 1]);
    }

    /// 64 cycles is the largest a sequence can carry, and it encodes as zero.
    #[test]
    fn sixty_four_cycles_encode_as_zero() {
        let request = SequenceRequest::new(vec![Sequence::output(bits![0; 64]).unwrap()]).unwrap();

        let mut buffer = [0u8; 16];
        request.to_bytes(&mut buffer).unwrap();

        assert_eq!(buffer[1], 0);
    }

    /// Only the captured sequences appear in the response, and their bits arrive least
    /// significant first.
    #[test]
    fn only_captured_sequences_are_parsed_back() {
        let request = SequenceRequest::new(vec![
            Sequence::output(bits![1, 1, 1]).unwrap(),
            Sequence::input(3).unwrap(),
        ])
        .unwrap();

        let SequenceResponse(_, bits) = request.parse_response(&[0x00, 0b101]).unwrap();

        assert_eq!(bits, bits![1, 0, 1]);
    }

    #[test]
    fn a_truncated_response_is_an_error() {
        let request = SequenceRequest::new(vec![Sequence::input(8).unwrap()]).unwrap();

        assert!(request.parse_response(&[0x00]).is_err());
    }
}
