//! Driving an IEEE 1149.7 target over two wires from the host.
//!
//! CMSIS-DAP has no cJTAG command, but it does not need one. The two things a two-pin target
//! wants are an escape sequence, which is TMSC edges with TCKC held high, and the OScan1 frame,
//! which is two cycles the host drives and one the target drives. `DAP_SWJ_Pins` makes a pin
//! edge without clocking, and `DAP_SWD_Sequence` clocks with SWDIO either driven or released.
//! SWCLK is TCKC and SWDIO is TMSC, so both land on the right wires.
//!
//! This is bench instrumentation, not a probe backend. Nothing here is wired into attaching.

use bitvec::prelude::*;

use super::{
    CmsisDap,
    commands::{
        CmsisDapError,
        jtag::sequence::{Sequence as JtagSequence, SequenceRequest as JtagSequenceRequest},
        swd::sequence::{Sequence as SwdSequence, SequenceRequest as SwdSequenceRequest},
        swj::sequence::SequenceRequest as SwjSequenceRequest,
    },
};
use crate::probe::DebugProbeError;

/// `STFMT`, Store Scan Format (SWCU185G Table 6-4).
const STFMT: usize = 3;

/// The `OSCAN1` operand of `STFMT`.
const OSCAN1: usize = 9;

/// `STC2`, Store Conditional 2 bit (SWCU185G Table 6-4).
const STC2: usize = 2;

/// The `STC2` operand that sets APFC to the standard pin function, which is four-pin JTAG.
const APFC_STANDARD_PINS: usize = 9;

/// Clocks held with TMS high to reach Test-Logic-Reset from anywhere.
const TAP_RESET_CLOCKS: usize = 5;

/// JTAG bits per `DAP_SWD_Sequence` request.
///
/// Each bit costs three bytes of request: an info byte and a data byte for the two cycles the
/// host drives, and an info byte for the one it releases. Twenty of them fit inside the 64-byte
/// report a version 1 probe is entitled to use, with the command and count bytes paid for.
const FRAMES_PER_REQUEST: usize = 20;

impl CmsisDap {
    /// Bring an IEEE 1149.7 target up in OScan1.
    ///
    /// cJTAG commands ride on ordinary scan activity (SWCU185G 6.2.1). Two zero-bit scans and a
    /// one-bit scan lock the control level, after which each command is a pair of DR scans whose
    /// **number of clocks in Shift-DR is the value**: `STFMT` is opcode 3 and `OSCAN1` is operand
    /// 9. probe-rs already drives the same window with 2 and 9, which is `STC2` selecting the
    /// standard pin function — the four-pin switch in `ti::sequences::icepick::ctag_to_jtag`.
    ///
    /// The TAP reset in front is what makes the inert opcode free: a reset loads IDCODE, which
    /// 6.2.1 accepts, so no instruction has to be shifted and TDI never has to be driven.
    ///
    /// Leaves the target TAP in Run-Test/Idle.
    ///
    /// A run of five or more TMS-high clocks afterwards returns the target to its power-up
    /// two-pin state, so anything that resets the TAP undoes this.
    pub fn cjtag_activate(&mut self) -> Result<(), DebugProbeError> {
        tracing::debug!("cJTAG: selecting OScan1");
        self.cjtag_command(STFMT, OSCAN1)
    }

    /// Switch the target to four-pin JTAG, the same way `ctag_to_jtag` does.
    ///
    /// This exists to test the half of the problem that OScan1 sits on top of. The command window
    /// and the clocking are shared with [`Self::cjtag_activate`]; only the opcode differs. If an
    /// ordinary JTAG scan finds the chain after this, the window opened and the bits arrived, and
    /// anything still wrong is in the OScan1 framing rather than underneath it.
    pub fn cjtag_switch_to_four_pin(&mut self) -> Result<(), DebugProbeError> {
        tracing::debug!("cJTAG: switching to four-pin");
        self.cjtag_command(STC2, APFC_STANDARD_PINS)
    }

    /// Open the command window and issue one command as its two DR scans.
    fn cjtag_command(&mut self, cp0: usize, cp1: usize) -> Result<(), DebugProbeError> {
        let mut tms = TapPattern::from_test_logic_reset();
        tms.zero_bit_scan();
        tms.zero_bit_scan();
        tms.dr_scan(1);
        tms.dr_scan(cp0);
        tms.dr_scan(cp1);

        let (bytes, count) = tms.finish();
        self.cjtag_drive_bits(&bytes, count)?;

        Ok(())
    }

    /// Read the first TAP's IDCODE over four-pin JTAG, without resetting the TAP.
    ///
    /// A chain scan cannot be used as a detector here: it resets the TAP, and five TMS-high
    /// clocks put a 1149.7 adapter back in its power-up two-pin state, undoing the very switch
    /// it is meant to be checking. This navigates from Run-Test/Idle instead, which is where
    /// [`Self::cjtag_switch_to_four_pin`] leaves it, and reads the register the reset selected.
    pub fn cjtag_four_pin_idcode(&mut self, reset_first: bool) -> Result<BitVec, DebugProbeError> {
        let mut sequences = Vec::new();

        // Nothing here knows where the last caller left the TAP, and from Test-Logic-Reset the
        // navigation below goes nowhere: TMS high holds that state, and the two low clocks only
        // reach Run-Test/Idle. Resetting makes the start known — at the cost of putting a 1149.7
        // adapter back in two-pin, which is why it is the caller's choice.
        if reset_first {
            sequences.push(JtagSequence::new(
                TAP_RESET_CLOCKS as u8,
                false,
                true,
                [0; 8],
            )?);
            sequences.push(JtagSequence::new(1, false, false, [0; 8])?);
        }

        sequences.extend([
            // Run-Test/Idle to Shift-DR.
            JtagSequence::new(1, false, true, [0; 8])?,
            JtagSequence::new(2, false, false, [0; 8])?,
            // Enough for several TAPs: the bits come out of the one nearest TDO first, so a
            // 32-bit read sees whichever device is last in the chain and nothing else.
            JtagSequence::new(64, true, false, [0; 8])?,
            JtagSequence::new(64, true, false, [0; 8])?,
        ]);

        Ok(self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?)
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
}

/// A TMS pattern for the TAP, built scan by scan.
///
/// cJTAG commands are counted in clocks spent in Shift-DR, so the pattern is the payload and
/// getting a state wrong is the whole bug. Every method here starts and ends in Run-Test/Idle, so
/// they compose, and the tests walk the result through the TAP model rather than trusting the
/// counting.
struct TapPattern {
    tms: Vec<bool>,
}

impl TapPattern {
    /// Start by resetting the TAP, which also leaves IDCODE in the instruction register.
    fn from_test_logic_reset() -> Self {
        let mut tms = vec![true; TAP_RESET_CLOCKS];
        tms.push(false);
        Self { tms }
    }

    /// A scan that reaches Update-DR without ever entering Shift-DR, which is what increments
    /// the control level (SWCU185G 6.2.1).
    ///
    /// It is the zero-clock case of [`Self::dr_scan`], which is why the standard counts command
    /// parts "from 0 to 31 clocks".
    fn zero_bit_scan(&mut self) {
        self.dr_scan(0);
    }

    /// A DR scan spending `clocks` clocks in Shift-DR, from Run-Test/Idle and back to it.
    ///
    /// The count is the payload of a cJTAG command, so it has to mean what probe-rs's own
    /// lowering means by an n-bit exchange: enter Shift-DR, clock `clocks` bits with TMS low
    /// except the last, which exits.
    fn dr_scan(&mut self, clocks: usize) {
        self.tms.extend([true, false]);
        self.tms.extend(std::iter::repeat_n(false, clocks));
        self.tms.extend([true, true, false]);
    }

    /// The pattern packed for `DAP_SWJ_Sequence`, least significant bit of each byte first.
    fn finish(self) -> (Vec<u8>, u8) {
        let count = u8::try_from(self.tms.len()).expect("a cJTAG command pattern is short");

        let mut bytes = vec![0u8; self.tms.len().div_ceil(8)];
        for (index, level) in self.tms.iter().enumerate() {
            if *level {
                bytes[index / 8] |= 1 << (index % 8);
            }
        }

        (bytes, count)
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

    use crate::probe::jtag::FullTapState;

    /// Walk a pattern through the TAP model and report the state after each clock.
    fn walk(pattern: TapPattern) -> Vec<FullTapState> {
        let mut state = FullTapState::TestLogicReset;
        pattern
            .tms
            .iter()
            .map(|tms| {
                state = state.step(*tms);
                state
            })
            .collect()
    }

    /// The control level only counts a scan that reaches Update-DR without shifting, so a zero
    /// bit scan that touches Shift-DR silently locks the level one command early.
    #[test]
    fn a_zero_bit_scan_captures_without_shifting() {
        let mut pattern = TapPattern::from_test_logic_reset();
        pattern.zero_bit_scan();

        let states = walk(pattern);

        assert!(states.contains(&FullTapState::CaptureDr), "never captured");
        assert!(states.contains(&FullTapState::UpdateDr), "never updated");
        assert!(!states.contains(&FullTapState::ShiftDr), "entered Shift-DR");
        assert_eq!(states.last(), Some(&FullTapState::RunTestIdle));
    }

    /// The clock count in Shift-DR *is* the command, so this is the assertion the whole
    /// activation rests on.
    #[test]
    fn a_dr_scan_spends_exactly_its_count_in_shift_dr() {
        for clocks in 0..=31 {
            let mut pattern = TapPattern::from_test_logic_reset();
            pattern.dr_scan(clocks);

            let states = walk(pattern);
            let shifted = states
                .iter()
                .filter(|state| **state == FullTapState::ShiftDr)
                .count();

            assert_eq!(shifted, clocks, "{clocks} clocks asked for");
            assert_eq!(states.last(), Some(&FullTapState::RunTestIdle));
        }
    }

    /// The way into Shift-DR is probe-rs's to define, not mine to hand-derive — the proven
    /// four-pin path uses it and this has to agree.
    #[test]
    fn the_scan_enters_shift_dr_the_way_the_rest_of_the_tree_does() {
        use crate::probe::TapState;

        let mut pattern = TapPattern { tms: Vec::new() };
        pattern.dr_scan(4);

        let path = TapState::RunTestIdle.path_to(TapState::ShiftDr);
        assert_eq!(&pattern.tms[..path.len()], path);
    }

    /// The reset has to be long enough from any starting state, and has to end in Run-Test/Idle
    /// rather than in Test-Logic-Reset, where a scan would go nowhere.
    #[test]
    fn the_reset_lands_in_run_test_idle() {
        let states = walk(TapPattern::from_test_logic_reset());
        assert_eq!(states.last(), Some(&FullTapState::RunTestIdle));
    }

    /// What the activation actually sends, end to end.
    #[test]
    fn the_activation_locks_the_level_then_sends_two_command_parts() {
        let mut pattern = TapPattern::from_test_logic_reset();
        pattern.zero_bit_scan();
        pattern.zero_bit_scan();
        pattern.dr_scan(1);
        pattern.dr_scan(STFMT);
        pattern.dr_scan(OSCAN1);

        let (bytes, count) = TapPattern {
            tms: pattern.tms.clone(),
        }
        .finish();

        // Reset 6, two zero-bit scans 5 each, the locking scan 6, then 8 and 14 for the two
        // command parts. A change here is a change to what the target is told.
        assert_eq!(count, 44);
        assert_eq!(bytes.len(), 6);

        let states = walk(pattern);
        assert_eq!(states.last(), Some(&FullTapState::RunTestIdle));
    }

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
