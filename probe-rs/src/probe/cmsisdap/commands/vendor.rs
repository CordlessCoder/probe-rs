//! Vendor commands of the Verge firmware, which frames JTAG in IEEE 1149.7 OScan formats.
//!
//! Any other probe answers these IDs with `DAP_Invalid` or its own vendor reply, which the magic
//! in the info reply tells apart.

use super::{CommandId, Request, SendError, Status};
use crate::probe::ScanFormat;

/// The info reply opens with this, then a version byte, a capability byte and the build.
const MAGIC: &[u8] = b"CJTAG";

/// The firmware frames OScan1 after an activation the host drove (control mode 2).
const CAP_OSCAN1_ENTER: u8 = 1 << 4;

/// The reply version that appends the formats the probe frames, after a board byte.
const VERSION_SCAN_FORMATS: u8 = 4;

/// The TAP state numbering control mode 3 takes: Run-Test/Idle.
const TAP_RUN_TEST_IDLE: u8 = 1;

/// What the firmware says about itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CjtagInfo {
    /// The layout version of this reply.
    pub version: u8,
    /// The capability bits.
    pub capabilities: u8,
    /// The build, as `DAP_Info` reports it.
    pub firmware: String,
    /// Bit n set for each SCNFMT n the probe frames in; zero before version 4.
    pub scan_formats: u32,
}

impl CjtagInfo {
    /// Whether the probe frames OScan1 after an activation the host drives.
    pub fn frames_after_host_activation(&self) -> bool {
        self.capabilities & CAP_OSCAN1_ENTER != 0
    }

    /// Whether the probe frames `format` after an activation the host drives.
    pub fn frames(&self, format: ScanFormat) -> bool {
        // Mode 2 frames OScan1 on every version that has it, mask or not.
        self.frames_after_host_activation()
            && (format == ScanFormat::OScan1 || self.scan_formats & 1 << format.number() != 0)
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
        // Fields after the build string are read only from the version that added them: a HID
        // reply is padded, so an older one has bytes there too.
        let scan_formats = if *version >= VERSION_SCAN_FORMATS {
            let formats = build
                .get(*len as usize + 1..)
                .and_then(|after_board| after_board.first_chunk())
                .ok_or(SendError::NotEnoughData)?;
            u32::from_le_bytes(*formats)
        } else {
            0
        };
        Ok(Some(CjtagInfo {
            version: *version,
            capabilities: *capabilities,
            firmware: String::from_utf8_lossy(firmware).into_owned(),
            scan_formats,
        }))
    }
}

/// Turns the firmware's two-pin framing on or off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CjtagControlRequest {
    /// Four-wire JTAG clocks.
    Off,
    /// Frame every JTAG clock in OScan1, after an activation the host drove.
    Frame,
    /// Frame every JTAG clock in this format, after the host switched the target to it, with the
    /// TAP in Run-Test/Idle.
    FrameFormat(ScanFormat),
}

impl Request for CjtagControlRequest {
    const COMMAND_ID: CommandId = CommandId::CjtagControl;

    type Response = Status;

    fn to_bytes(&self, buffer: &mut [u8]) -> Result<usize, SendError> {
        let request: &[u8] = match self {
            CjtagControlRequest::Off => &[0],
            CjtagControlRequest::Frame => &[2],
            CjtagControlRequest::FrameFormat(format) => &[3, format.number(), TAP_RUN_TEST_IDLE],
        };
        buffer
            .get_mut(..request.len())
            .ok_or(SendError::NotEnoughData)?
            .copy_from_slice(request);
        Ok(request.len())
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
        assert_eq!(info.scan_formats, 0);
        assert!(info.frames(ScanFormat::OScan1));
        assert!(!info.frames(ScanFormat::OScan2));
    }

    #[test]
    fn version_four_lists_the_formats_after_the_board() {
        let mut reply = b"CJTAG".to_vec();
        reply.extend([4, 0x1d, 3]);
        reply.extend(b"abc");
        reply.push(0);
        reply.extend(0x0000_0600u32.to_le_bytes());

        let info = CjtagInfoRequest.parse_response(&reply).unwrap().unwrap();

        assert_eq!(info.scan_formats, 0x600);
        assert!(info.frames(ScanFormat::OScan2));
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

    #[test]
    fn another_format_is_mode_three_from_run_test_idle() {
        let mut buffer = [0xAA; 4];
        let request = CjtagControlRequest::FrameFormat(ScanFormat::OScan2);
        assert_eq!(request.to_bytes(&mut buffer).unwrap(), 3);
        assert_eq!(buffer[..3], [3, 10, 1]);
    }
}
