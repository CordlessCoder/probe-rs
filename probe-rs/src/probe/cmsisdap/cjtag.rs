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
        self, CmsisDapError, RequestError,
        general::connect::{ConnectRequest, ConnectResponse},
        jtag::sequence::{Sequence as JtagSequence, SequenceRequest as JtagSequenceRequest},
        swd::sequence::{Sequence as SwdSequence, SequenceRequest as SwdSequenceRequest},
        swj::sequence::SequenceRequest as SwjSequenceRequest,
    },
};
use probe_rs_target::ScanChainElement;

use crate::probe::{BitSequence, DebugProbeError, JtagBatch, JtagChain, TapState, WireProtocol};

/// `STFMT`, Store Scan Format (SWCU185G Table 6-4).
const STFMT: u64 = 3;

/// The `OSCAN1` operand of `STFMT`.
const OSCAN1: u64 = 9;

/// `STC2`, Store Conditional 2 bit (SWCU185G Table 6-4).
const STC2: u64 = 2;

/// The `STC2` operand that sets APFC to the standard pin function, which is four-pin JTAG.
const APFC_STANDARD_PINS: u64 = 9;

/// TDI data for a sequence that only cares about TMS.
///
/// All ones, not all zeros: `DAP_JTAG_Sequence` drives TDI from these bytes, so zeros would shift
/// zeros into the instruction register. The cJTAG command window wants an inert opcode there, and
/// BYPASS is all ones (SWCU185G 6.2.1) — which is also what the four-pin path that works on this
/// part sends.
const TDI_HIGH: [u8; 8] = [0xFF; 8];

/// The ICEPick's instruction register length.
const ICEPICK_IR_LEN: usize = 6;

/// BYPASS, the inert opcode the command window wants loaded first (SWCU185G 6.2.1).
const ICEPICK_BYPASS: u32 = 0x3F;

/// The ICEPick's IDCODE instruction.
const ICEPICK_IDCODE: u32 = 0x04;

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
    ///
    /// This drives the chain through probe-rs's own JTAG operations, the same ones
    /// `ti::sequences::icepick::ctag_to_jtag` uses to switch this part to four pin. Hand-rolling
    /// the TMS pattern reproduced the right bits on the wire and still did not take, so the only
    /// difference left worth removing was the machinery itself.
    ///
    /// The clock count of each DR scan is the command's value (SWCU185G 6.2.1).
    fn cjtag_command(&mut self, cp0: u64, cp1: u64) -> Result<(), DebugProbeError> {
        // What `debug_port_setup` sends before it builds the ICEPick: a line reset that puts the
        // SWJ pins into a known state. Without it the command window is being opened from
        // whatever the previous caller left behind.
        self.send_swj_sequences(SwjSequenceRequest::new(
            &0x0007_FFFF_FFFF_FFFFu64.to_le_bytes(),
            51,
        )?)?;

        let mut chain = JtagChain::new(self);
        // Deliberately no `set_chain` here. `Icepick::new` opens the window with the chain
        // unset and only configures it after scanning, so an instruction shift sends exactly the
        // bits given with no padding. Setting it first pads them and the command changes.
        // A reset leaves an inert opcode in the instruction register, and BYPASS is shifted in
        // after it because that is what the working path does.
        let mut batch = JtagBatch::new();
        chain.tap_reset(&mut batch);
        chain.run(batch)?;

        let bypass = BitSequence::from_bytes(&ICEPICK_BYPASS.to_le_bytes(), ICEPICK_IR_LEN);
        let mut batch = JtagBatch::new();
        chain.shift_ir(&mut batch, &bypass);
        chain.run_test_idle(&mut batch, 0);
        chain.run(batch)?;

        // Two zero-bit scans raise the control level; a scan that reaches Update-DR without
        // touching Shift-DR is what counts, and entering Pause-DR is one way there.
        for _ in 0..2 {
            let mut batch = JtagBatch::new();
            batch.enter(TapState::PauseDr);
            batch.enter(TapState::RunTestIdle);
            chain.run(batch)?;
        }

        // The first Shift-DR locks the level, then the command parts follow.
        for clocks in [1, cp0, cp1] {
            let mut batch = JtagBatch::new();
            let bits = BitSequence::from_u64(clocks as usize, 0xff);
            chain.exchange_dr(&mut batch, &bits);
            chain.run_test_idle(&mut batch, 0);
            chain.run(batch)?;
        }

        Ok(())
    }

    /// Read the first TAP's IDCODE through probe-rs's own chain operations.
    ///
    /// The same shape as `icepick::scan_jtag`: select IDCODE, then exchange a data register of
    /// the expected width. Going through `JtagChain` rather than raw sequences keeps the chain's
    /// own state tracking in step, which raw sequences quietly bypass.
    pub fn cjtag_chain_idcode(&mut self) -> Result<BitVec, DebugProbeError> {
        let mut chain = JtagChain::new(self);
        // The chain operations pad IR against the configured chain, so it has to be told what
        // is on it before any of them. `Icepick::new` does the same once it has scanned; at this
        // point only the ICEPick is attached, which is what the power-up state gives us.
        chain.set_chain(&[ScanChainElement {
            name: Some("ICEPICK".to_owned()),
            ir_len: Some(ICEPICK_IR_LEN as u8),
        }]);

        // BYPASS first, then IDCODE, which is what `ctag_to_jtag` does on its way out: the
        // BYPASS load is there so later state transitions cannot disturb the instruction
        // register. Shifting IDCODE straight after the command skips that.
        for instruction in [ICEPICK_BYPASS, ICEPICK_IDCODE] {
            let bits = BitSequence::from_bytes(&instruction.to_le_bytes(), ICEPICK_IR_LEN);
            let mut batch = JtagBatch::new();
            chain.shift_ir(&mut batch, &bits);
            chain.run_test_idle(&mut batch, 0);
            chain.run(batch)?;
        }

        let mut batch = JtagBatch::new();
        let handle = chain.exchange_dr(&mut batch, &BitSequence::from_u64(32, 0));
        chain.run_test_idle(&mut batch, 0);
        let mut results = chain.run(batch)?;

        let bits = results.take(handle).map_err(|error| {
            DebugProbeError::Other(format!("no IDCODE came back from the chain: {error:?}"))
        })?;

        Ok(bits.iter().collect())
    }

    /// Read the chain's TDO over four-pin JTAG from Run-Test/Idle, without resetting the TAP.
    ///
    /// A chain scan cannot serve as a detector here: it resets the TAP, and that returns a 1149.7
    /// adapter to two-pin, undoing the switch being checked.
    pub fn cjtag_four_pin_idcode(&mut self, reset_first: bool) -> Result<BitVec, DebugProbeError> {
        let mut sequences = Vec::new();

        if reset_first {
            sequences.push(JtagSequence::new(6, false, true, TDI_HIGH)?);
            sequences.push(JtagSequence::new(1, false, false, TDI_HIGH)?);
        }

        sequences.extend([
            JtagSequence::new(1, false, true, TDI_HIGH)?,
            JtagSequence::new(2, false, false, TDI_HIGH)?,
            JtagSequence::new(64, true, false, TDI_HIGH)?,
            JtagSequence::new(64, true, false, TDI_HIGH)?,
        ]);

        Ok(self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?)
    }

    /// Reconnect the probe in SWD mode, keeping the target where it is.
    ///
    /// The command window rides on JTAG operations, and the OScan1 read needs the probe able to
    /// release the wire, which only the SWD commands do. `DAP_Connect` reconfigures the probe's
    /// pins and clocks nothing, so the target does not notice.
    pub fn cjtag_reconnect_as_swd(&mut self) -> Result<(), DebugProbeError> {
        let used = commands::send_command(&mut self.device, &ConnectRequest::Swd).and_then(
            |v| match v {
                ConnectResponse::SuccessfulInitForSWD => Ok(WireProtocol::Swd),
                ConnectResponse::SuccessfulInitForJTAG => Ok(WireProtocol::Jtag),
                ConnectResponse::InitFailed => {
                    Err(CmsisDapError::ErrorResponse(RequestError::InitFailed {
                        protocol: Some(WireProtocol::Swd),
                    }))
                }
            },
        )?;

        self.protocol = Some(used);
        tracing::debug!("cJTAG: reconnected as {used}");

        Ok(())
    }

    /// Reconnect the probe in JTAG mode. The counterpart of [`Self::cjtag_reconnect_as_swd`],
    /// for checking whether reconnecting is what disturbs the target.
    pub fn cjtag_reconnect_as_jtag(&mut self) -> Result<(), DebugProbeError> {
        commands::send_command(&mut self.device, &ConnectRequest::Jtag)
            .map(|_: ConnectResponse| ())?;
        self.protocol = Some(WireProtocol::Jtag);
        Ok(())
    }

    /// Shift `frames` of `(tms, tdi)` through an activated target, one OScan1 frame each.
    ///
    /// Returns one TDO bit per frame, in order. Needs the probe connected in SWD mode, because
    /// releasing the wire for the target's cycle is what `DAP_SWD_Sequence` exists to do.
    pub fn cjtag_shift(&mut self, frames: &[(bool, bool)]) -> Result<BitVec, DebugProbeError> {
        let mut tdo = BitVec::new();

        for chunk in frames.chunks(FRAMES_PER_REQUEST) {
            let request = SwdSequenceRequest::new(oscan1_frames(chunk)?)?;
            tdo.extend_from_bitslice(&self.send_swd_sequences(request)?);
        }

        Ok(tdo)
    }

    /// Load `instruction` into the instruction register over OScan1, from Run-Test/Idle and
    /// back to it.
    ///
    /// A TAP reset would select IDCODE for free and also put a 1149.7 adapter back in two-pin,
    /// undoing the activation — so the instruction has to be shifted in. OScan1 carries TDI, so
    /// this is the one thing two-pin can do that the activation itself cannot.
    pub fn cjtag_shift_ir(&mut self, instruction: u32) -> Result<(), DebugProbeError> {
        let mut frames = vec![(true, false), (true, false), (false, false), (false, false)];

        for bit in 0..ICEPICK_IR_LEN {
            let last = bit == ICEPICK_IR_LEN - 1;
            frames.push((last, instruction >> bit & 1 == 1));
        }

        frames.extend([(true, false), (false, false)]);

        self.cjtag_shift(&frames)?;

        Ok(())
    }

    /// Read `bits` out of the selected data register over OScan1, from Run-Test/Idle.
    ///
    /// Stays in Shift-DR at the end rather than exiting, which costs nothing here and keeps the
    /// frame count equal to the bit count.
    pub fn cjtag_read_dr(&mut self, bits: usize) -> Result<BitVec, DebugProbeError> {
        let mut frames = vec![(true, false), (false, false), (false, false)];
        frames.extend(std::iter::repeat_n((false, false), bits));

        let tdo = self.cjtag_shift(&frames)?;

        // The first three frames are the walk to Shift-DR and carry no register data.
        Ok(tdo[3..].to_bitvec())
    }

    /// Bench probe: clock `count` TCK cycles with TMS high, as one JTAG sequence and nothing
    /// else, so a capture can be compared against the number asked for.
    pub fn cjtag_emit_tms_run(&mut self, count: u8) -> Result<(), DebugProbeError> {
        let sequences = vec![JtagSequence::new(count, false, true, [0; 8])?];
        self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;
        Ok(())
    }

    /// Bench probe: emit TMS-high runs of each length in `counts`, separated by one low clock,
    /// as a single request. The capture then shows every length against what was asked for.
    pub fn cjtag_sweep_tms_runs(&mut self, counts: &[u8]) -> Result<(), DebugProbeError> {
        let mut sequences = Vec::new();

        for count in counts {
            sequences.push(JtagSequence::new(*count, false, true, [0; 8])?);
            sequences.push(JtagSequence::new(1, false, false, [0; 8])?);
        }

        self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;

        Ok(())
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
