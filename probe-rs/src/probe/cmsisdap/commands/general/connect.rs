use super::super::{CommandId, Request, SendError};

#[derive(Clone, Copy, Debug, docsplay::Display)]
pub enum ConnectRequest {
    /// the default debug port
    DefaultPort = 0x00,
    /// SWD
    Swd = 0x01,
    /// JTAG
    Jtag = 0x02,
}

impl Request for ConnectRequest {
    const COMMAND_ID: CommandId = CommandId::Connect;

    type Response = ConnectResponse;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        buffer[0] = *self as u8;
        Ok(1)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        match buffer[0] {
            0 => Ok(ConnectResponse::InitFailed),
            1 => Ok(ConnectResponse::SuccessfulInitForSWD),
            2 => Ok(ConnectResponse::SuccessfulInitForJTAG),
            other => Err(SendError::ConnectResponseError(other)),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum ConnectResponse {
    InitFailed = 0x00,
    SuccessfulInitForSWD = 0x01,
    SuccessfulInitForJTAG = 0x02,
}

#[cfg(test)]
mod tests {
    use super::super::super::RequestError;
    use super::*;

    #[test]
    fn a_refused_connect_names_the_port() {
        let message = |request| RequestError::InitFailed { request }.to_string();
        assert_eq!(
            message(ConnectRequest::Swd),
            "The probe could not initialize SWD"
        );
        assert_eq!(
            message(ConnectRequest::DefaultPort),
            "The probe could not initialize the default debug port"
        );
    }
}
