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
        general::{
            connect::{ConnectRequest, ConnectResponse},
            execute::PinBurstRequest,
        },
        jtag::sequence::{Sequence as JtagSequence, SequenceRequest as JtagSequenceRequest},
        swd::sequence::{Sequence as SwdSequence, SequenceRequest as SwdSequenceRequest},
        swj::{
            pins::{SWJPinsRequestBuilder, SWJPinsResponse},
            sequence::SequenceRequest as SwjSequenceRequest,
        },
    },
};
use probe_rs_target::ScanChainElement;

use crate::architecture::arm::Pins;
use crate::probe::{BitSequence, DebugProbeError, JtagBatch, JtagChain, TapState, WireProtocol};

/// How long each escape-sequence pin level is held.
const ESCAPE_EDGE_DWELL: std::time::Duration = std::time::Duration::from_millis(1);

/// Frames held with TMS high to reset the TAP from inside OScan1.
///
/// The measured working read uses nine; ten is the same thing with a frame to spare.
const OSCAN1_RESET_FRAMES: usize = 10;

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
        // The scan sequence a working two-pin activation puts on the wire, read off a capture:
        // after the scan that locks the control level, these clock counts. Pairing them gives
        // `STC1` with SEDGE set, then `STFMT` selecting OScan1, then two more commands, the last
        // of which pairs across the group boundary.
        self.cjtag_scans(&[1, 1, STFMT, OSCAN1, 1, 2, 2])
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
        self.cjtag_scans(&[cp0, cp1])
    }

    /// Open the command window once, then run `scans` as data-register scans of those clock
    /// counts, which is the form a cJTAG command takes.
    ///
    /// Taking clock counts rather than command pairs is what lets a sequence read off a working
    /// capture be replayed exactly, including a scan that pairs across a group boundary.
    fn cjtag_scans(&mut self, scans: &[u64]) -> Result<(), DebugProbeError> {
        self.cjtag_open_window()?;
        self.cjtag_dr_scans(
            &std::iter::once(1)
                .chain(scans.iter().copied())
                .collect::<Vec<_>>(),
        )
    }

    /// Reset the TAP, load the inert opcode, and raise the control level with two zero-bit
    /// scans. Leaves the window open; the level locks on the next Shift-DR.
    fn cjtag_open_window(&mut self) -> Result<(), DebugProbeError> {
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

        Ok(())
    }

    /// Run `scans` as data-register scans of those clock counts, through an open window.
    fn cjtag_dr_scans(&mut self, scans: &[u64]) -> Result<(), DebugProbeError> {
        let mut chain = JtagChain::new(self);

        for clocks in scans.iter().copied() {
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

    /// Bring the target up in OScan1 the way the working two-pin path does: an escape sequence
    /// and the command group, repeated.
    ///
    /// Both halves are measured. The escape is six TMSC edges with TCKC held high, driven from
    /// pin writes and confirmed on the wire at the pacing the working path uses. The group is the
    /// scan sequence that path sends, also read off a capture, and this reproduces it scan for
    /// scan. What the capture additionally shows is that the two alternate, five times over.
    pub fn cjtag_activate_interleaved(
        &mut self,
        edges: usize,
        passes: usize,
    ) -> Result<(), DebugProbeError> {
        // The window is opened once. The working path does not reload the inert opcode before
        // each repeat — it escapes and sends the group again, and nothing else.
        self.cjtag_open_window()?;

        for pass in 0..passes {
            tracing::debug!("cJTAG: interleaved activation, pass {pass}");
            self.cjtag_escape_fast(edges)?;
            self.cjtag_dr_scans(&[1, 1, 1, STFMT, OSCAN1, 1, 2, 2])?;
        }

        Ok(())
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

    /// Send the four-pin switch as OScan1 frames, through a target already in OScan1.
    ///
    /// Every failure so far is the target not driving TMSC, and a probe cannot tell that apart
    /// from a target that never understood the frames in the first place. This separates them,
    /// because it needs no answer on TMSC at all: if the target receives these frames it leaves
    /// OScan1 for four-pin, and the ordinary four-pin read afterwards says so.
    ///
    /// An answer means the receive path works and only the target's drive-back is missing. A
    /// silent target means the frames never landed, and nothing about turnaround is implicated.
    pub fn cjtag_four_pin_over_oscan1(&mut self) -> Result<(), DebugProbeError> {
        tracing::debug!("cJTAG: four-pin command over OScan1 frames");

        for group in command_window_scans(&[STC2, APFC_STANDARD_PINS]) {
            self.cjtag_shift(&group)?;
        }

        Ok(())
    }

    /// Drive the same generated frames over four pins, one TCK cycle per frame.
    ///
    /// This is the control for [`Self::cjtag_four_pin_over_oscan1`]. A negative there could just
    /// as easily mean the frames are wrong as that the target is not listening, and the only way
    /// to tell is to run them where the target is known to listen. Selecting OScan1 this way and
    /// watching the target go quiet on four pins says the generator builds a real command window.
    ///
    /// `select_oscan` picks which command the window carries. Sending the four-pin one instead
    /// is the positive half of the control: the target has to keep answering, which a generator
    /// that merely wedged the TAP could not produce.
    pub fn cjtag_window_frames_four_pin(
        &mut self,
        select_oscan: bool,
    ) -> Result<(), DebugProbeError> {
        tracing::debug!("cJTAG: command from generated frames, over four pins");

        let groups = if select_oscan {
            command_window_scans(&[1, 1, STFMT, OSCAN1, 1, 2, 2])
        } else {
            command_window_scans(&[STC2, APFC_STANDARD_PINS])
        };

        for group in groups {
            for chunk in group.chunks(64) {
                let mut sequences = Vec::with_capacity(chunk.len());
                for &(tms, tdi) in chunk {
                    let data = if tdi { TDI_HIGH } else { [0u8; 8] };
                    sequences.push(JtagSequence::new(1, false, tms, data)?);
                }
                self.send_jtag_sequences(JtagSequenceRequest::new(sequences)?)?;
            }
        }

        Ok(())
    }

    /// Clock `cycles` extra TCKC cycles before the first OScan1 frame, slipping the frame grid.
    ///
    /// An OScan1 frame is three cycles and the target counts them from the activation. If it
    /// starts that count a cycle away from where the host does, every frame it receives is
    /// misread, it never reaches a state where it answers, and TMSC keeps whatever level the
    /// host last drove. On a capture that is indistinguishable from a target that is simply not
    /// driving, so the only way to tell the two apart is to move the grid and look again.
    ///
    /// Three cycles return the grid to where it started, so offsets 0, 1 and 2 cover it.
    ///
    /// TMSC stays low throughout: five TMS-high clocks put a 1149.7 adapter back in its
    /// power-up two-pin state, which would undo the activation being tested.
    pub fn cjtag_pad_clocks(&mut self, cycles: usize) -> Result<(), DebugProbeError> {
        if cycles == 0 {
            return Ok(());
        }

        tracing::debug!("cJTAG: {cycles} padding clocks before the first frame");
        self.cjtag_drive_swd_bits(&vec![0u8; cycles.div_ceil(8)], cycles)
    }

    /// Bring the target up in OScan1 through the escape-sequence route.
    ///
    /// This is what the working two-pin path does, and it is not the documented command window.
    /// A capture of it shows a burst of TMSC edges with TCKC held high, which no clocked command
    /// can express, followed by clocked patterns carrying the command levels.
    ///
    /// The patterns are transcribed from that capture. Needs the probe in SWD mode, because the
    /// bits go out as SWD sequences on the same two wires.
    pub fn cjtag_activate_by_escape(&mut self, edges: usize) -> Result<(), DebugProbeError> {
        tracing::debug!("cJTAG: escape activation, {edges} edges");

        // Test-Logic-Reset, then one clock into Run-Test/Idle: the last of the 201 bits is low.
        let mut reset = [0xFFu8; 26];
        reset[25] = 0x00;
        self.cjtag_drive_swd_bits(&reset, 201)?;

        self.cjtag_pins(Some(false), None)?;
        self.cjtag_pins(None, Some(false))?;
        self.cjtag_pins(Some(true), None)?;
        for edge in 0..edges {
            self.cjtag_pins(None, Some(edge % 2 == 0))?;
        }

        // The command levels, ending with the scan format set to OScan1.
        self.cjtag_drive_swd_bits(&[0x00; 4], 32)?;
        self.cjtag_drive_swd_bits(&[0xAD, 0x65, 0x59, 0x16, 0x16, 0x80, 0x01], 50)?;
        self.cjtag_drive_swd_bits(&[0x01, 0xD6, 0x00], 18)?;
        self.cjtag_drive_swd_bits(&[0x1B], 6)?;

        Ok(())
    }

    /// Drive the escape sequence as one packet, so the edges land microseconds apart.
    ///
    /// `DAP_SWJ_Pins` one per packet puts edges a millisecond apart, which is a USB round trip.
    /// The working two-pin path places them about 1.1 us apart, and that is only reachable by
    /// handing the probe several writes at once.
    pub fn cjtag_escape_fast(&mut self, edges: usize) -> Result<(), DebugProbeError> {
        let mut writes = Vec::with_capacity(edges + 3);

        let mut clock = Pins(0);
        clock.set_swclk_tck(true);
        let mut data = Pins(0);
        data.set_swdio_tms(true);

        // TCKC low, TMSC low, TCKC high: the state an escape sequence starts from.
        writes.push((Pins(0), clock));
        writes.push((Pins(0), data));
        writes.push((clock, clock));

        for edge in 0..edges {
            let level = if edge % 2 == 0 { data } else { Pins(0) };
            writes.push((level, data));
        }

        self.send_pin_burst(PinBurstRequest::new(writes)?)?;

        Ok(())
    }

    fn send_pin_burst(&mut self, request: PinBurstRequest) -> Result<(), DebugProbeError> {
        commands::send_command(&mut self.device, &request)?;
        Ok(())
    }

    /// Bench probe: drive TMSC low, then release it for `cycles` clocks.
    ///
    /// This is how to see whether the probe really stops driving the wire for the cycle OScan1
    /// gives the target. A released line with any pull-up on it rises; a line still driven low
    /// stays flat. Driving low first is what makes the two cases tell apart — releasing a line
    /// that is already high shows nothing either way.
    pub fn cjtag_release_test(&mut self, cycles: usize) -> Result<BitVec, DebugProbeError> {
        let mut low = BitVec::new();
        low.extend(std::iter::repeat_n(false, 8));

        let sequences = vec![SwdSequence::output(&low)?, SwdSequence::input(cycles)?];

        Ok(self.send_swd_sequences(SwdSequenceRequest::new(sequences)?)?)
    }

    /// Bench probe: drive just the escape sequence, with no clocked patterns around it, so a
    /// capture shows the pin writes alone.
    pub fn cjtag_escape_only(&mut self, edges: usize) -> Result<(), DebugProbeError> {
        self.cjtag_pins(Some(false), None)?;
        self.cjtag_pins(None, Some(false))?;
        self.cjtag_pins(Some(true), None)?;

        for edge in 0..edges {
            self.cjtag_pins(None, Some(edge % 2 == 0))?;
        }

        Ok(())
    }

    /// Clock `bits` of `data` on TMSC as SWD sequences, least significant bit of each byte first.
    fn cjtag_drive_swd_bits(&mut self, data: &[u8], bits: usize) -> Result<(), DebugProbeError> {
        let mut driven = BitVec::<u8, Lsb0>::from_slice(data);
        driven.truncate(bits);

        let mut sequences = Vec::new();
        for chunk in driven.chunks(64) {
            let mut part = BitVec::new();
            part.extend(chunk.iter().map(|b| *b));
            sequences.push(SwdSequence::output(&part)?);
        }

        self.send_swd_sequences(SwdSequenceRequest::new(sequences)?)?;

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

        // Each edge has to be held long enough for the target to count it. The working path
        // paces these about a millisecond apart, which a USB round trip already is.
        std::thread::sleep(ESCAPE_EDGE_DWELL);

        Ok(())
    }

    /// Read the first TAP's IDCODE over OScan1, the way the J-Link does it.
    ///
    /// Taken frame for frame from a capture of a working two-pin read: a TAP reset carried out
    /// *as OScan1 frames*, then a zero-bit instruction scan, then the walk to Shift-DR.
    ///
    /// Resetting the TAP this way is safe even though a run of TMS-high clocks normally returns
    /// a 1149.7 adapter to its power-up two-pin state. In OScan1 a frame is three cycles and TMS
    /// is only the middle one, so TMSC never carries the long high run the adapter watches for.
    ///
    /// The reset also makes an instruction shift unnecessary: IEEE 1149.1 leaves IDCODE selected
    /// after one.
    pub fn cjtag_read_idcode(&mut self) -> Result<BitVec, DebugProbeError> {
        let mut frames = vec![(true, false); OSCAN1_RESET_FRAMES];

        // Run-Test/Idle, then Select-DR, Capture-DR, Shift-DR.
        frames.extend([
            (false, false),
            (true, false),
            (false, false),
            (false, false),
        ]);

        // The register itself, with TMS low so the TAP stays in Shift-DR.
        frames.extend(std::iter::repeat_n((false, false), 32));

        self.cjtag_shift(&frames)
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

/// The cJTAG command window as one frame list per request: the reset and inert opcode, the two
/// scans that raise the control level, then `scans` as data-register scans of those clock counts.
///
/// Split rather than concatenated because the working path sends each scan as its own request and
/// lands them 100 us and more apart, where one batch puts them 10 us apart. A window driven from
/// one batch does not take, which is the likeliest reading of an earlier bench result where a
/// hand-rolled pattern was byte-identical on the wire and still did nothing.
///
/// The leading scan is prepended here for the same reason `cjtag_scans` prepends it: the command
/// pairs start one scan after the level locks.
fn command_window_scans(scans: &[u64]) -> Vec<Vec<(bool, bool)>> {
    let mut opening = vec![(true, true); OSCAN1_RESET_FRAMES];
    opening.push((false, true));
    shift_ir_frames(ICEPICK_BYPASS, &mut opening);

    let mut groups = vec![opening];
    for _ in 0..2 {
        let mut group = Vec::new();
        zero_bit_scan_frames(&mut group);
        groups.push(group);
    }
    for clocks in std::iter::once(1).chain(scans.iter().copied()) {
        let mut group = Vec::new();
        dr_scan_frames(clocks as usize, &mut group);
        groups.push(group);
    }

    groups
}

/// Frames for one data-register scan of `clocks` clocks in Shift-DR, from Run-Test/Idle and back.
///
/// The clock count is the command's value, so the count here is the payload (SWCU185G 6.2.1).
fn dr_scan_frames(clocks: usize, frames: &mut Vec<(bool, bool)>) {
    frames.push((true, true));
    frames.push((false, true));
    frames.push((false, true));
    for clock in 0..clocks {
        frames.push((clock == clocks - 1, true));
    }
    frames.push((true, true));
    frames.push((false, true));
}

/// Frames for a scan that reaches Update-DR without ever entering Shift-DR, which is what raises
/// the cJTAG control level. Pause-DR is one way there.
fn zero_bit_scan_frames(frames: &mut Vec<(bool, bool)>) {
    for tms in [true, false, true, false, true, true, false] {
        frames.push((tms, true));
    }
}

/// Frames that shift `instruction` into the instruction register, from Run-Test/Idle and back.
fn shift_ir_frames(instruction: u32, frames: &mut Vec<(bool, bool)>) {
    frames.push((true, true));
    frames.push((true, true));
    frames.push((false, true));
    frames.push((false, true));
    for bit in 0..ICEPICK_IR_LEN {
        let value = instruction >> bit & 1 == 1;
        frames.push((bit == ICEPICK_IR_LEN - 1, value));
    }
    frames.push((true, true));
    frames.push((false, true));
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
