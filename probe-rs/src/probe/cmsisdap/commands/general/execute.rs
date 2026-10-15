/// Implementation of the DAP_EXECUTECOMMANDS command
use super::super::{CmsisDapError, CommandId, Request, SendError};
use crate::architecture::arm::Pins;

/// Several pin writes carried out back to back by the probe, from one packet.
///
/// One `DAP_SWJ_Pins` per packet means one USB round trip per pin edge, which puts edges about a
/// millisecond apart. An IEEE 1149.7 escape sequence is counted in edges rather than timed, but a
/// target samples them, and the working two-pin path places them microseconds apart. Batching is
/// the only way to approach that from the host: the probe runs the commands with nothing between
/// them.
///
/// The standard's own example of this command is two `DAP_SWJ_Pins` writes in one packet.
#[derive(Clone, Debug)]
pub struct PinBurstRequest {
    /// Each write's output levels and the mask of pins it drives.
    writes: Vec<(Pins, Pins)>,

    /// Microseconds of `DAP_Delay` between writes, or zero for none.
    ///
    /// The standard's own example of this command pairs `DAP_SWJ_Pins` with `DAP_Delay`, which
    /// is the only way to pace edges from the host: the `Pin Wait` field is a readback timeout
    /// for an open-drain line to stabilise, not a delay.
    delay_us: u16,
}

impl PinBurstRequest {
    /// `delay_us` of `DAP_Delay` between each write, or zero for none.
    pub(crate) fn paced(writes: Vec<(Pins, Pins)>, delay_us: u16) -> Result<Self, CmsisDapError> {
        // Each write may be followed by a delay, so the command count has to fit a byte too.
        let commands = if delay_us == 0 {
            writes.len()
        } else {
            writes.len() * 2
        };
        if writes.is_empty() || commands > u8::MAX as usize {
            return Err(CmsisDapError::TooMuchData);
        }

        Ok(Self { writes, delay_us })
    }

    /// Bytes one delay takes: the command id and a little-endian microsecond count.
    const DELAY_LEN: usize = 3;

    /// Bytes one write takes: the command id, the levels, the mask, and a four-byte wait.
    const WRITE_LEN: usize = 7;
}

impl Request for PinBurstRequest {
    const COMMAND_ID: CommandId = CommandId::ExecuteCommands;

    type Response = ();

    /*
    | BYTE | BYTE **| BYTE ************|
    > 0x7F | NumCmd | Command Requests |
    |******|********|++++++++++++++++++|
     */
    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        let paced = self.delay_us != 0;
        let per_write = Self::WRITE_LEN + if paced { Self::DELAY_LEN } else { 0 };
        let len = 1 + self.writes.len() * per_write;
        if buffer.len() < len {
            return Err(SendError::NotEnoughData);
        }

        buffer[0] = (self.writes.len() * if paced { 2 } else { 1 }) as u8;

        for (index, (output, select)) in self.writes.iter().enumerate() {
            let at = 1 + index * per_write;

            buffer[at] = CommandId::SwjPins as u8;
            buffer[at + 1] = output.0;
            buffer[at + 2] = select.0;
            // No wait: this field is a readback timeout, not a delay.
            buffer[at + 3..at + 7].copy_from_slice(&0u32.to_le_bytes());

            if paced {
                buffer[at + 7] = CommandId::Delay as u8;
                buffer[at + 8..at + 10].copy_from_slice(&self.delay_us.to_le_bytes());
            }
        }

        Ok(len)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        // Each write answers with its command id and the pins read back. Only the count is worth
        // checking: a probe that ran fewer commands than asked did not produce the edges.
        let executed = *buffer.first().ok_or(SendError::NotEnoughData)? as usize;
        let expected = self.writes.len() * if self.delay_us == 0 { 1 } else { 2 };

        if executed != expected {
            return Err(SendError::UnexpectedAnswer);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The layout the standard gives: a count, then each write complete with its own command id.
    #[test]
    fn writes_are_concatenated_behind_a_count() {
        let request =
            PinBurstRequest::paced(vec![(Pins(0x02), Pins(0x02)), (Pins(0x00), Pins(0x02))], 0)
                .unwrap();

        let mut buffer = [0u8; 32];
        let written = request.to_bytes(&mut buffer).unwrap();

        assert_eq!(written, 15);
        assert_eq!(&buffer[..3], &[2, CommandId::SwjPins as u8, 0x02]);
        assert_eq!(buffer[8], CommandId::SwjPins as u8);
    }

    /// Pacing puts a `DAP_Delay` after each write, and the count covers both commands.
    #[test]
    fn a_paced_burst_interleaves_delays() {
        let request = PinBurstRequest::paced(vec![(Pins(0x02), Pins(0x02))], 3).unwrap();

        let mut buffer = [0u8; 32];
        let written = request.to_bytes(&mut buffer).unwrap();

        assert_eq!(written, 11);
        assert_eq!(buffer[0], 2, "one write and one delay");
        assert_eq!(buffer[8], CommandId::Delay as u8);
        assert_eq!(&buffer[9..11], &3u16.to_le_bytes());
        // The probe answers for both commands.
        assert!(request.parse_response(&[2]).is_ok());
        assert!(request.parse_response(&[1]).is_err());
    }

    /// A probe that ran fewer commands than asked did not make all the edges, and an escape
    /// sequence is counted in edges, so a short burst is a different sequence.
    #[test]
    fn a_short_count_is_rejected() {
        let request = PinBurstRequest::paced(vec![(Pins(0), Pins(2)); 4], 0).unwrap();

        assert!(request.parse_response(&[4]).is_ok());
        assert!(request.parse_response(&[3]).is_err());
    }
}
