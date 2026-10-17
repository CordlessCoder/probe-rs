/// Implementation of the DAP_EXECUTECOMMANDS command
use super::super::{CmsisDapError, CommandId, Request, SendError};
use crate::architecture::arm::Pins;

/// One command of a [`PinProgramRequest`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinCommand {
    /// A `DAP_SWJ_Pins` write: the output levels, and the mask of the pins it drives.
    Write(Pins, Pins),
    /// A `DAP_Delay` of this many microseconds. Never zero: ARM's firmware runs a zero delay as
    /// 2^32 iterations.
    Delay(u16),
}

impl PinCommand {
    /// Bytes this command takes in the request: the command id, then a write's levels, mask and
    /// four-byte wait, or a delay's little-endian microsecond count.
    pub(crate) fn len(&self) -> usize {
        match self {
            PinCommand::Write(..) => 7,
            PinCommand::Delay(_) => 3,
        }
    }
}

/// Pin writes and delays in any order, carried out back to back from one packet.
#[derive(Clone, Debug)]
pub struct PinProgramRequest {
    commands: Vec<PinCommand>,
}

impl PinProgramRequest {
    /// Bytes the count takes ahead of the commands.
    pub(crate) const HEADER_LEN: usize = 1;

    pub(crate) fn new(commands: Vec<PinCommand>) -> Result<Self, CmsisDapError> {
        if commands.is_empty()
            || commands.len() > u8::MAX as usize
            || commands.contains(&PinCommand::Delay(0))
        {
            return Err(CmsisDapError::TooMuchData);
        }
        Ok(Self { commands })
    }
}

impl Request for PinProgramRequest {
    const COMMAND_ID: CommandId = CommandId::ExecuteCommands;

    type Response = ();

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        let len = Self::HEADER_LEN + self.commands.iter().map(PinCommand::len).sum::<usize>();
        if buffer.len() < len {
            return Err(SendError::NotEnoughData);
        }

        buffer[0] = self.commands.len() as u8;
        let mut at = Self::HEADER_LEN;
        for command in &self.commands {
            match command {
                PinCommand::Write(output, select) => {
                    buffer[at] = CommandId::SwjPins as u8;
                    buffer[at + 1] = output.0;
                    buffer[at + 2] = select.0;
                    // No wait: this field is a readback timeout, not a delay.
                    buffer[at + 3..at + 7].copy_from_slice(&0u32.to_le_bytes());
                }
                PinCommand::Delay(us) => {
                    buffer[at] = CommandId::Delay as u8;
                    buffer[at + 1..at + 3].copy_from_slice(&us.to_le_bytes());
                }
            }
            at += command.len();
        }

        Ok(len)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        // A probe that ran fewer commands than asked did not make all the edges.
        let executed = *buffer.first().ok_or(SendError::NotEnoughData)? as usize;
        if executed != self.commands.len() {
            return Err(SendError::UnexpectedAnswer);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Writes and delays keep their order, and the count covers both.
    #[test]
    fn a_program_interleaves_writes_and_delays_as_given() {
        let request = PinProgramRequest::new(vec![
            PinCommand::Write(Pins(0x01), Pins(0x01)),
            PinCommand::Delay(200),
            PinCommand::Write(Pins(0x00), Pins(0x02)),
        ])
        .unwrap();

        let mut buffer = [0u8; 32];
        assert_eq!(request.to_bytes(&mut buffer).unwrap(), 1 + 7 + 3 + 7);
        assert_eq!(buffer[0], 3);
        assert_eq!(&buffer[1..4], &[CommandId::SwjPins as u8, 0x01, 0x01]);
        assert_eq!(&buffer[8..11], &[CommandId::Delay as u8, 200, 0]);
        assert_eq!(&buffer[11..14], &[CommandId::SwjPins as u8, 0x00, 0x02]);
        assert!(request.parse_response(&[3]).is_ok());
        assert!(request.parse_response(&[2]).is_err());
    }

    #[test]
    fn a_zero_delay_is_refused() {
        assert!(PinProgramRequest::new(vec![PinCommand::Delay(0)]).is_err());
    }
}
