use super::super::{CommandId, Request, SendError};
use super::{connect::ConnectRequest, disconnect::DisconnectRequest};

/// A command whose request carries at most one byte after its ID, and whose reply exactly one.
/// [`ExecuteCommandsRequest`] carries only these, so it can split the replies without parsing
/// them, and a batch's request is never longer than its reply.
pub(crate) trait OneByteCommand: Request {}

impl OneByteCommand for ConnectRequest {}
impl OneByteCommand for DisconnectRequest {}

/// A [`OneByteCommand`] without its response type, so one request can hold different commands.
trait Batched {
    fn command(&self) -> CommandId;
    fn write(&self, buffer: &mut [u8]) -> Result<usize, SendError>;
}

impl<R: OneByteCommand> Batched for R {
    fn command(&self) -> CommandId {
        Request::command(self)
    }

    fn write(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        self.to_bytes(buffer)
    }
}

/// `DAP_ExecuteCommands`: the probe runs the commands in order and answers them in one reply. Only
/// for a probe that reports atomic commands.
#[derive(Default)]
pub(crate) struct ExecuteCommandsRequest {
    commands: Vec<Box<dyn Batched>>,
}

impl ExecuteCommandsRequest {
    pub(crate) fn push(&mut self, command: impl OneByteCommand + 'static) {
        self.commands.push(Box::new(command));
    }
}

impl Request for ExecuteCommandsRequest {
    fn command(&self) -> CommandId {
        CommandId::ExecuteCommands
    }

    /// The byte each command answered, in order.
    type Response = Vec<u8>;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        // The probe does not check that its replies fit in the packet, so the host has to.
        let count = self.commands.len();
        if count > usize::from(u8::MAX) || 1 + 2 * count > buffer.len() {
            return Err(SendError::TooManyCommands { count });
        }
        buffer[0] = count as u8;
        let mut len = 1;
        for command in &self.commands {
            buffer[len] = command.command().id();
            len += 1 + command.write(&mut buffer[len + 1..])?;
        }
        Ok(len)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        let (&count, replies) = buffer.split_first().ok_or(SendError::NotEnoughData)?;
        if usize::from(count) != self.commands.len() {
            return Err(SendError::UnexpectedAnswer);
        }
        let (replies, _) = replies.as_chunks::<2>();
        let replies = replies
            .get(..self.commands.len())
            .ok_or(SendError::NotEnoughData)?;
        self.commands
            .iter()
            .zip(replies)
            .map(|(command, &[id, reply])| {
                if id == command.command().id() {
                    Ok(reply)
                } else {
                    Err(SendError::CommandIdMismatch(id, command.command()))
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_reply_to_another_command_is_an_error() {
        let mut request = ExecuteCommandsRequest::default();
        request.push(DisconnectRequest {});
        request.push(ConnectRequest::Jtag);
        // A probe that does not know a command answers 0xFF in its place.
        assert!(
            request
                .parse_response(&[2, 0x03, 0x00, 0xFF, 0x02])
                .is_err()
        );
    }

    #[test]
    fn the_replies_have_to_fit_in_one_packet() {
        let mut request = ExecuteCommandsRequest::default();
        for _ in 0..31 {
            request.push(DisconnectRequest {});
        }
        // The packet holds the reply's ID, its count and two bytes per command.
        let mut buffer = [0; 63];
        assert!(request.to_bytes(&mut buffer).is_ok());
        request.push(DisconnectRequest {});
        assert!(request.to_bytes(&mut buffer).is_err());
    }
}
