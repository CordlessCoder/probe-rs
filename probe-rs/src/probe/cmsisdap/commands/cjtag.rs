//! The cJTAG commands proposed for CMSIS-DAP in ARM-software/CMSIS-DAP#8: `DAP_JTAG_Format` and
//! `DAP_JTAG_Escape`.
//!
//! ARM has not assigned their IDs yet. Until it does they go out under the vendor IDs of the Verge
//! firmware, which implements them, and a probe announces them through Verge's info command.

use super::general::execute_commands::OneByteCommand;
use super::{CommandId, Request, SendError, Status, VendorCommandId};

/// `DAP_JTAG_Format` on Verge, until ARM assigns its ID.
const FORMAT: VendorCommandId = VendorCommandId::new(0x95).expect("in the vendor range");
/// `DAP_JTAG_Escape` on Verge, until ARM assigns its ID.
const ESCAPE: VendorCommandId = VendorCommandId::new(0x96).expect("in the vendor range");
/// Verge's info command. Other firmware gives this ID other meanings (DAPLink's sets its UART up),
/// so it is only sent to a probe that names itself Verge's.
const VERGE_INFO: VendorCommandId = VendorCommandId::new(0x82).expect("in the vendor range");

/// The `DAP_Info` vendor name of the Verge firmware.
pub(crate) const VERGE_VENDOR: &str = "Escapement Project";

/// Selects how the probe encodes the JTAG commands: conventional JTAG (0), or OScan `n` on two pins
/// (`n`). On a live JTAG connection it takes effect at once. While disconnected the probe holds it,
/// touching no pins, and the next `DAP_Connect` of the JTAG port comes up in it and uses it up; 0
/// clears it, and so does `DAP_Disconnect`. Refused on an SWD connection. A probe rejects a format
/// it does not have and keeps the one it has.
#[derive(Clone, Copy, Debug)]
pub struct FormatRequest {
    pub(crate) format: u8,
}

impl Request for FormatRequest {
    fn command(&self) -> CommandId {
        CommandId::Vendor(FORMAT)
    }

    type Response = Status;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        buffer[0] = self.format;
        Ok(1)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        Status::from_byte(*buffer.first().ok_or(SendError::NotEnoughData)?)
    }
}

impl OneByteCommand for FormatRequest {}

/// An escape: TMSC goes to `level` with TCKC low, then makes `edges` transitions while TCKC is
/// high for one bit period, outside any frame.
#[derive(Clone, Copy, Debug)]
pub struct EscapeRequest {
    pub(crate) edges: u8,
    pub(crate) level: bool,
}

impl Request for EscapeRequest {
    fn command(&self) -> CommandId {
        CommandId::Vendor(ESCAPE)
    }

    type Response = Status;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        buffer[0] = self.edges;
        buffer[1] = u8::from(self.level);
        Ok(2)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        Status::from_byte(*buffer.first().ok_or(SendError::NotEnoughData)?)
    }
}

/// Asks Verge whether it has the cJTAG commands.
#[derive(Clone, Copy, Debug)]
pub struct VergeInfoRequest;

impl VergeInfoRequest {
    /// The reply opens with this, then a layout version.
    const MAGIC: &[u8] = b"CJTAG";
    /// The first layout version whose `DAP_JTAG_Format` sent while disconnected selects the format
    /// of the next `DAP_Connect`. Version 7 had the commands but refused that.
    const VERSION: u8 = 8;
}

impl Request for VergeInfoRequest {
    fn command(&self) -> CommandId {
        CommandId::Vendor(VERGE_INFO)
    }

    /// Whether the firmware has the cJTAG commands.
    type Response = bool;

    fn to_bytes(&self, _buffer: &mut [u8]) -> Result<usize, SendError> {
        Ok(0)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        Ok(buffer
            .strip_prefix(Self::MAGIC)
            .and_then(|rest| rest.first())
            .is_some_and(|&version| version >= Self::VERSION))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verge_info_needs_the_magic_and_version_eight() {
        let parse = |reply: &[u8]| VergeInfoRequest.parse_response(reply).unwrap();
        assert!(parse(b"CJTAG\x08\x00"));
        assert!(!parse(b"CJTAG\x07\x1d"));
        assert!(!parse(b"CJTA"));
        assert!(!parse(&[0; 8]));
    }
}
