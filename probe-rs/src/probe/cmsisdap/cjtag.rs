//! Driving an IEEE 1149.7 target over two wires from the host.
//!
//! CMSIS-DAP has no cJTAG command, but it does not need one. The two things a two-pin target
//! wants are an escape sequence, which is TMSC edges with TCKC held high, and the OScan1 frame,
//! which is two cycles the host drives and one the target drives. `DAP_SWJ_Pins` makes a pin
//! edge without clocking, and `DAP_SWD_Sequence` clocks with SWDIO either driven or released.
//! SWCLK is TCKC and SWDIO is TMSC, so both land on the right wires.
//!
//! This is bench instrumentation, not a probe backend. Nothing here is wired into attaching.

use std::time::Duration;

use bitvec::prelude::*;

use super::{
    CmsisDap,
    commands::{
        self, CmsisDapError,
        swd::sequence::{Sequence as SwdSequence, SequenceRequest as SwdSequenceRequest},
        swj::{
            pins::{SWJPinsRequestBuilder, SWJPinsResponse},
            sequence::SequenceRequest as SwjSequenceRequest,
        },
    },
};
use crate::probe::DebugProbeError;

/// TMS edges in the IEEE 1149.7 selection escape sequence.
const SELECTION_ESCAPE_EDGES: usize = 6;

/// How long each escape-sequence pin level is held.
///
/// SEGGER's own software paces these about a millisecond apart, which is what says the sequence
/// counts edges rather than timing them. A USB round trip is already in that range.
const ESCAPE_EDGE_DWELL: Duration = Duration::from_millis(1);

/// JTAG bits per `DAP_SWD_Sequence` request.
///
/// Each bit costs three bytes of request: an info byte and a data byte for the two cycles the
/// host drives, and an info byte for the one it releases. Twenty of them fit inside the 64-byte
/// report a version 1 probe is entitled to use, with the command and count bytes paid for.
const FRAMES_PER_REQUEST: usize = 20;

impl CmsisDap {
    /// Bring an IEEE 1149.7 target up in OScan1.
    ///
    /// The sequence is transcribed from a capture of SEGGER's software activating a CC13xx, with
    /// its J-Link firmware parameters dropped — those configure the probe and never reach the
    /// wire. What is left is a TAP reset, the selection escape, and the command-level writes that
    /// end in the documented `WriteOScan1(0x1B, 0, 6)`.
    ///
    /// Leaves the target TAP in Run-Test/Idle.
    ///
    /// A run of five or more TMS-high clocks afterwards returns the target to its power-up
    /// two-pin state, so anything that resets the TAP undoes this.
    pub fn cjtag_activate(&mut self) -> Result<(), DebugProbeError> {
        tracing::debug!("cJTAG: activating OScan1");

        // Test-Logic-Reset, then one clock into Run-Test/Idle: the last of the 201 bits is low.
        let mut reset = [0xFFu8; 26];
        reset[25] = 0x00;
        self.cjtag_drive_bits(&reset, 201)?;

        // An escape sequence is TMS edges while TCK is held high, which no clocked command can
        // express. These go out as individual pin writes instead.
        self.cjtag_pins(Some(false), None)?;
        self.cjtag_pins(None, Some(false))?;
        self.cjtag_pins(Some(true), None)?;
        for edge in 0..SELECTION_ESCAPE_EDGES {
            self.cjtag_pins(None, Some(edge % 2 == 0))?;
        }

        // The command levels, ending with the scan format set to OScan1.
        self.cjtag_drive_bits(&[0x00; 4], 32)?;
        self.cjtag_drive_bits(&[0xAD, 0x65, 0x59, 0x16, 0x16, 0x80, 0x01], 50)?;
        self.cjtag_drive_bits(&[0x01, 0xD6, 0x00], 18)?;
        self.cjtag_drive_bits(&[0x1B], 6)?;

        Ok(())
    }

    /// Shift `frames` of `(tms, tdi)` through an activated target, one OScan1 frame each.
    ///
    /// Returns one TDO bit per frame, in order.
    ///
    /// OScan1 spends three TCKC cycles on a JTAG bit: the host drives the inverse of TDI, then
    /// TMS, then releases the wire for the target to answer on.
    pub fn cjtag_shift(&mut self, frames: &[(bool, bool)]) -> Result<BitVec, DebugProbeError> {
        let mut tdo = BitVec::new();

        for chunk in frames.chunks(FRAMES_PER_REQUEST) {
            let request = SwdSequenceRequest::new(oscan1_frames(chunk)?)?;
            tdo.extend_from_bitslice(&self.send_swd_sequences(request)?);
        }

        Ok(tdo)
    }

    /// Whether this probe answers `DAP_SWD_Sequence` at all.
    ///
    /// No capability bit reports it, so the only way to find out is to send one. A probe that
    /// does not know the command reports that rather than running it.
    pub fn cjtag_swd_sequence_supported(&mut self) -> Result<bool, DebugProbeError> {
        let request = SwdSequenceRequest::new(vec![SwdSequence::input(1)?])?;

        match self.send_swd_sequences(request) {
            Ok(_) => Ok(true),
            Err(error) => {
                tracing::debug!("DAP_SWD_Sequence was refused: {error}");
                Ok(false)
            }
        }
    }

    /// Drive `bits` of `data` on TMSC, least significant bit of each byte first.
    fn cjtag_drive_bits(&mut self, data: &[u8], bits: u8) -> Result<(), DebugProbeError> {
        self.send_swj_sequences(SwjSequenceRequest::new(data, bits)?)?;
        Ok(())
    }

    /// Set TCKC and TMSC directly, leaving a pin alone where the level is `None`.
    fn cjtag_pins(
        &mut self,
        swclk: Option<bool>,
        swdio: Option<bool>,
    ) -> Result<(), DebugProbeError> {
        let mut builder = SWJPinsRequestBuilder::new();

        if let Some(level) = swclk {
            builder.swclk_tck(level);
        }
        if let Some(level) = swdio {
            builder.swdio_tms(level);
        }

        commands::send_command(&mut self.device, &builder.build()).map(|_: SWJPinsResponse| ())?;

        // Each edge has to be held long enough for the target to count it.
        std::thread::sleep(ESCAPE_EDGE_DWELL);

        Ok(())
    }
}

/// Turn `(tms, tdi)` pairs into the sequences one OScan1 frame each takes.
///
/// A frame is three TCKC cycles: the host drives the inverse of TDI and then TMS, and releases
/// the wire for the third so the target can answer on it (SWCU185G Table 6-3).
fn oscan1_frames(frames: &[(bool, bool)]) -> Result<Vec<SwdSequence>, CmsisDapError> {
    let mut sequences = Vec::with_capacity(frames.len() * 2);

    for &(tms, tdi) in frames {
        let mut driven = BitVec::new();
        driven.push(!tdi);
        driven.push(tms);

        sequences.push(SwdSequence::output(&driven)?);
        sequences.push(SwdSequence::input(1)?);
    }

    Ok(sequences)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe::cmsisdap::commands::Request;

    /// What one frame puts on the wire, read out of the encoded request.
    fn encoded(tms: bool, tdi: bool) -> Vec<u8> {
        let request = SwdSequenceRequest::new(oscan1_frames(&[(tms, tdi)]).unwrap()).unwrap();
        let mut buffer = [0u8; 16];
        let written = request.to_bytes(&mut buffer).unwrap();
        buffer[..written].to_vec()
    }

    /// TDI goes out inverted and TMS does not, which is the half of the frame a bench run
    /// cannot tell apart from a wiring fault.
    #[test]
    fn tdi_is_inverted_and_tms_is_not() {
        // Two sequences; the first drives two cycles, bit 0 = !tdi, bit 1 = tms.
        assert_eq!(encoded(false, false), vec![2, 2, 0b01, 0x80 | 1]);
        assert_eq!(encoded(false, true), vec![2, 2, 0b00, 0x80 | 1]);
        assert_eq!(encoded(true, false), vec![2, 2, 0b11, 0x80 | 1]);
        assert_eq!(encoded(true, true), vec![2, 2, 0b10, 0x80 | 1]);
    }

    /// The released cycle has to be its own sequence, because that is the only way the probe is
    /// told to stop driving. Two sequences per frame, always.
    #[test]
    fn every_frame_releases_the_wire_once() {
        let sequences = oscan1_frames(&[(false, false); 8]).unwrap();
        assert_eq!(sequences.len(), 16);
    }

    /// A request has to fit the smallest report a probe may use, or a long shift silently
    /// overruns the buffer it is written into.
    #[test]
    fn a_full_request_fits_a_64_byte_report() {
        let request =
            SwdSequenceRequest::new(oscan1_frames(&[(false, false); FRAMES_PER_REQUEST]).unwrap())
                .unwrap();

        let mut buffer = [0u8; 63];
        let written = request.to_bytes(&mut buffer).unwrap();

        assert!(
            written <= 63,
            "{written} bytes does not fit a 64-byte report"
        );
    }
}
