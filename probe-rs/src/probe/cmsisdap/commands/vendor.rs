//! Vendor commands of the mculink-cjtag firmware, which frames JTAG as IEEE 1149.7 OScan1.
//!
//! Any other probe answers these IDs with `DAP_Invalid` or its own vendor reply, which the magic
//! in the info reply tells apart.

use super::{CommandId, Request, SendError, Status};

/// The info reply opens with this, then a version byte, a capability byte and the build.
const MAGIC: &[u8] = b"CJTAG";

/// The firmware frames after an activation the host drove (control mode 2).
const CAP_OSCAN1_ENTER: u8 = 1 << 4;

/// What the firmware says about itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CjtagInfo {
    /// The layout version of this reply.
    pub version: u8,
    /// The capability bits.
    pub capabilities: u8,
    /// The build, as `DAP_Info` reports it.
    pub firmware: String,
}

impl CjtagInfo {
    /// Whether the probe frames OScan1 after an activation the host drives.
    pub fn frames_after_host_activation(&self) -> bool {
        self.capabilities & CAP_OSCAN1_ENTER != 0
    }
}

/// Asks the probe whether it is the cJTAG firmware.
#[derive(Clone, Copy, Debug)]
pub struct CjtagInfoRequest;

impl Request for CjtagInfoRequest {
    const COMMAND_ID: CommandId = CommandId::CjtagInfo;

    /// `None` from a probe that answers the ID but is not this firmware.
    type Response = Option<CjtagInfo>;

    fn to_bytes(&self, _buffer: &mut [u8]) -> Result<usize, SendError> {
        Ok(0)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        let Some(rest) = buffer.strip_prefix(MAGIC) else {
            return Ok(None);
        };
        let [version, capabilities, len, build @ ..] = rest else {
            return Err(SendError::NotEnoughData);
        };
        let firmware = build.get(..*len as usize).ok_or(SendError::NotEnoughData)?;
        Ok(Some(CjtagInfo {
            version: *version,
            capabilities: *capabilities,
            firmware: String::from_utf8_lossy(firmware).into_owned(),
        }))
    }
}

/// Turns the firmware's OScan1 framing on or off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CjtagControlRequest {
    /// Four-wire JTAG clocks.
    Off,
    /// Frame every JTAG clock, after an activation the host drove.
    Frame,
}

impl Request for CjtagControlRequest {
    const COMMAND_ID: CommandId = CommandId::CjtagControl;

    type Response = Status;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        *buffer.first_mut().ok_or(SendError::NotEnoughData)? = match self {
            CjtagControlRequest::Off => 0,
            CjtagControlRequest::Frame => 2,
        };
        Ok(1)
    }

    fn parse_response(&self, buffer: &[u8]) -> Result<Self::Response, SendError> {
        Status::from_byte(*buffer.first().ok_or(SendError::NotEnoughData)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_info_reply_is_parsed_after_its_magic() {
        let mut reply = b"CJTAG".to_vec();
        reply.extend([2, 0x1d, 3]);
        reply.extend(b"abcXX");

        let info = CjtagInfoRequest.parse_response(&reply).unwrap().unwrap();

        assert_eq!(info.version, 2);
        assert!(info.frames_after_host_activation());
        assert_eq!(info.firmware, "abc");
    }

    #[test]
    fn another_firmware_answering_the_id_is_not_this_one() {
        assert_eq!(
            CjtagInfoRequest.parse_response(&[0, 0, 0, 0]).unwrap(),
            None
        );
    }

    #[test]
    fn framing_on_is_control_mode_two() {
        let mut buffer = [0xAA; 4];
        assert_eq!(CjtagControlRequest::Frame.to_bytes(&mut buffer).unwrap(), 1);
        assert_eq!(buffer[0], 2);
    }
}
