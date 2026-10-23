//! Activating two-pin JTAG on a TI CC13xx/CC26xx (SWCU185G 6.2 to 6.4, IEEE 1149.7-2022).
//!
//! This is the routine TI's XDS110 firmware runs, as a TI engineer posted it in TI E2E thread
//! 766675 ("CC1352P: How to enable cJTAG OSCAN1 mode on CC13xx and CC26xx device"), with three
//! additions. TI's routine starts from a powered-down JTAG domain and leaves out the power-up
//! wait the TRM requires. Here a reset escape and more wake clocks come first, so it also
//! attaches to a target a previous session left framing, or half woken. And where the probe
//! frames formats beyond OScan1, the target's TAP.7 controller is asked which it has, and switched
//! to one they share.
//!
//! The adapter powers up in two-pin JScan, where TMSC is TMS on every clock. Command scans then
//! select OScan1, and from there every JTAG clock is a frame. Between some steps the adapter's
//! control state machine (CSM) consumes clocks that the TAP never sees.

use std::ops::Range;
use std::time::Duration;

use crate::probe::{
    CjtagProbe, DebugProbeError, PinStep, ScanFormat, TapState, jtag::FullTapState,
};

/// TMSC edges with TCKC high that reset the adapter, whatever format it is in: 8 or more is a
/// reset escape. The J-Link sends 10.
const RESET_ESCAPE_EDGES: usize = 10;

/// TMS-high clocks that wake the JTAG power domain. ICEMelter wants 8 rising and 8 falling edges
/// within 4 ms (TRM 6.4); a domain left half woken needs more than 8.
const WAKE_CLOCKS: usize = 20;

/// The JTAG power domain's power-up time before it takes commands (TRM 6.4).
const POWER_UP: Duration = Duration::from_micros(200);

/// TAP.7 command opcodes (IEEE 1149.7-2022 Table 9-4).
const STMC: usize = 0;
const STFMT: usize = 3;
const MCM: usize = 6;
const SCNS: usize = 9;

/// STMC's Exit Control Level: closes the command window, which couples the TAP again.
const ECL: usize = 1;

/// SCNS's operand for a CR scan of CNFG0, the controller's configuration.
const CNFG0: usize = 2;

/// The shortest Check Packet: a preamble, a two-clock CP_END and a postamble (IEEE 1149.7-2022
/// 11.7.9).
const CHECK_PACKET_CLOCKS: usize = 4;

/// CNFG0's width.
const CNFG0_BITS: usize = 32;

/// The Shift-DR clocks of [`cr_scan`].
const CR_SCAN_SHIFT: Range<usize> = 3..3 + CNFG0_BITS;

/// One step of the activation.
#[derive(Debug, PartialEq, Eq)]
enum Step {
    /// Unframed clocks, one TMS bit each: what the TAP sees in JScan.
    Jscan(Vec<bool>),
    /// Unframed TMS-low clocks that the adapter's control state machine consumes.
    Csm(usize),
    /// OScan1 frames, one TMS bit each.
    Frames(Vec<bool>),
}

/// `count` TMS bits of `pattern`, least significant first: TI's `WriteTmsPattern`. Bits past
/// the pattern's width are low.
fn tms(pattern: u32, count: u32) -> Vec<bool> {
    (0..count)
        .map(|bit| pattern.checked_shr(bit).is_some_and(|rest| rest & 1 != 0))
        .collect()
}

/// TI's routine after its Test-Logic-Reset, from Run-Test/Idle, without the ECL that ends it. Every
/// DR scan's Shift-DR clock count is a command part, so one clock more or less is a different
/// command.
fn ti_steps() -> Vec<Step> {
    // A zero-bit scan from Run-Test/Idle or Update-DR, ending in Update-DR.
    let zbs = || [tms(0x05, 4), tms(0x03, 2)].concat();
    vec![
        // An IR scan of BYPASS, an inert opcode while the command window is open.
        Step::Jscan([tms(0x03, 3), tms(0, 64), tms(0x03, 3)].concat()),
        // Two zero-bit scans and a one-clock scan open the window and decouple the TAP.
        Step::Jscan([zbs(), zbs()].concat()),
        Step::Jscan([tms(0x05, 4), tms(0x0D, 4), tms(0x05, 4)].concat()),
        // STC1 1: sample TMSC on the rising edge.
        Step::Jscan([tms(0x0D, 4), tms(0x05, 4), tms(0x0D, 4), tms(0x05, 4)].concat()),
        // STFMT 9: OScan1, ending in Select-DR.
        Step::Jscan([tms(0x31, 6), tms(0x05, 4), tms(0x1C01, 13)].concat()),
        Step::Csm(4),
        // On to Pause-DR, then STMC 4: deselect every controller.
        Step::Frames(tms(0x02, 3)),
        Step::Frames([tms(0x03, 2), tms(0x15, 6), tms(0x08, 4), tms(0x01, 2)].concat()),
        Step::Csm(7),
        // MSC 0: make controller 0 a scan group candidate, which selects it.
        Step::Frames(tms(0x05, 4)),
        Step::Frames(
            [
                tms(0x01, 2),
                tms(0x40, 7),
                tms(0x01, 1),
                tms(0x05, 4),
                tms(0x03, 3),
            ]
            .concat(),
        ),
        Step::Csm(7),
    ]
}

/// A TAP.7 command from Run-Test/Idle back to it: two DR scans whose Shift-DR clock counts are the
/// opcode and the operand, each through Pause-DR as TI writes them.
fn command(opcode: usize, operand: usize) -> Vec<bool> {
    [command_part(opcode), command_part(operand), vec![false]].concat()
}

/// A DR scan of `clocks` Shift-DR clocks, from Run-Test/Idle or Update-DR to Update-DR.
fn command_part(clocks: usize) -> Vec<bool> {
    // On to Pause-DR and Exit2-DR, then through Shift-DR if there is anything to shift.
    let mut tms = vec![true, false, true, false, true];
    if clocks > 0 {
        tms.extend(std::iter::repeat_n(false, clocks));
        tms.push(true);
    }
    tms.push(true);
    tms
}

/// A DR scan of CNFG0 from Run-Test/Idle back to it.
fn cr_scan() -> Vec<bool> {
    [
        vec![true, false, false],
        vec![false; CNFG0_BITS - 1],
        vec![true, true, false],
    ]
    .concat()
}

/// The formats beyond OScan1 the activation switches to, fewest bits for a DAP scan first, which
/// is also the order they read fastest in on the probe firmware that frames them.
const SWITCH_TO: [ScanFormat; 3] = [ScanFormat::OScan6, ScanFormat::OScan5, ScanFormat::OScan2];

/// Whether CNFG0 (IEEE 1149.7-2022 Table 9-15) says the controller has `format`: class T4 or T5,
/// this revision of the standard, and the OSCANS bit of the format's pair. A read nothing drove,
/// all ones or all zeros, says no.
fn has_format(cnfg0: u32, format: ScanFormat) -> bool {
    let oscans = match format {
        ScanFormat::OScan1 => return true,
        ScanFormat::OScan2 => 22,
        ScanFormat::OScan5 => 23,
        ScanFormat::OScan6 => 24,
    };
    let class = cnfg0 & 0xF;
    let revision = cnfg0 >> 4 & 0xF;
    matches!(class, 4 | 5) && revision == 1 && cnfg0 & 1 << oscans != 0
}

/// The clocks of a scan packet outside Shift-xR (IEEE 1149.7-2022 Table 25-1).
fn scan_packet_clocks(format: ScanFormat) -> usize {
    match format {
        ScanFormat::OScan1 | ScanFormat::OScan5 => 3,
        ScanFormat::OScan2 | ScanFormat::OScan6 => 1,
    }
}

/// The pins as the activation drives them, tracking their levels so that TMSC moves only while
/// TCKC is low, except in an escape.
struct Wire<'a> {
    probe: &'a mut dyn CjtagProbe,
    steps: Vec<PinStep>,
    tckc: bool,
    tmsc: bool,
    /// The format the probe frames in, once the first frame has gone out.
    framing: Option<ScanFormat>,
}

impl<'a> Wire<'a> {
    /// Takes both pins high, as `DAP_Connect` leaves them. From a session that ended low, the
    /// clock edge is one more TMS-high clock.
    fn new(probe: &'a mut dyn CjtagProbe) -> Self {
        Self {
            probe,
            steps: vec![PinStep::Tmsc(true), PinStep::Tckc(true)],
            tckc: true,
            tmsc: true,
            framing: None,
        }
    }

    fn escape(&mut self, edges: usize) {
        if !self.tckc {
            self.tckc = true;
            self.steps.push(PinStep::Tckc(true));
        }
        for _ in 0..edges {
            self.tmsc = !self.tmsc;
            self.steps.push(PinStep::Tmsc(self.tmsc));
        }
    }

    /// Unframed clocks. While framing is on, the probe frames again afterwards.
    fn clock(&mut self, tms: &[bool]) -> Result<(), DebugProbeError> {
        for &bit in tms {
            self.steps.push(PinStep::Tckc(false));
            if bit != self.tmsc {
                self.tmsc = bit;
                self.steps.push(PinStep::Tmsc(bit));
            }
            self.steps.push(PinStep::Tckc(true));
        }
        self.tckc = true;
        if let Some(format) = self.framing {
            self.start_framing(format)?;
        }
        Ok(())
    }

    fn wait(&mut self, wait: Duration) {
        self.steps.push(PinStep::Wait(wait));
    }

    /// The format frames go out in. The first ones are OScan1.
    fn format(&self) -> ScanFormat {
        self.framing.unwrap_or(ScanFormat::OScan1)
    }

    /// Frames, one TMS bit each, returning TDO for each.
    fn frames(&mut self, tms: &[bool]) -> Result<Vec<bool>, DebugProbeError> {
        if self.framing.is_none() {
            self.start_framing(ScanFormat::OScan1)?;
            self.framing = Some(ScanFormat::OScan1);
        }
        let tdo = self.probe.cjtag_tms(tms)?;
        // A frame ends with TCKC low and TMSC back at its TMS bit.
        self.tckc = false;
        self.tmsc = tms.last().copied().unwrap_or(self.tmsc);
        Ok(tdo)
    }

    /// A command, from Run-Test/Idle back to it, in the current format.
    fn command(&mut self, opcode: usize, operand: usize) -> Result<(), DebugProbeError> {
        self.command_then(opcode, operand, self.format())
    }

    /// STFMT: the target frames in `format` once the command's Check Packet ends.
    fn set_format(&mut self, format: ScanFormat) -> Result<(), DebugProbeError> {
        self.command_then(STFMT, format.number().into(), format)
    }

    /// A command, then the scan packet the TAP does not act on and the Check Packet that follow
    /// it in an OScan format (IEEE 1149.7-2022 Table 21-5, case B). Both are in the format the
    /// command went out in; the probe frames `next` after them.
    fn command_then(
        &mut self,
        opcode: usize,
        operand: usize,
        next: ScanFormat,
    ) -> Result<(), DebugProbeError> {
        let current = self.format();
        self.frames(&command(opcode, operand))?;
        self.framing = Some(next);
        self.clock(&vec![
            false;
            scan_packet_clocks(current) + CHECK_PACKET_CLOCKS
        ])
    }

    /// Reads CNFG0 with a CR scan, inside the command window.
    fn read_cnfg0(&mut self) -> Result<u32, DebugProbeError> {
        // A CR scan reads the register only from the one conditional group member, and ones
        // while there may be more (IEEE 1149.7-2022 14.8.1.1.2). MCM 0 makes controller 0 it.
        self.command(MCM, 0)?;
        self.command(SCNS, CNFG0)?;
        let scan = cr_scan();
        let tdo = self.frames(&scan)?;
        let bits = tdo.get(CR_SCAN_SHIFT).ok_or_else(|| {
            DebugProbeError::Other(format!(
                "The probe returned {} TDO bits for {} clocks",
                tdo.len(),
                scan.len()
            ))
        })?;
        Ok(bits
            .iter()
            .rev()
            .fold(0, |value, &bit| value << 1 | u32::from(bit)))
    }

    /// Every frame starts with TCKC low.
    fn start_framing(&mut self, format: ScanFormat) -> Result<(), DebugProbeError> {
        self.steps.push(PinStep::Tckc(false));
        self.tckc = false;
        self.flush()?;
        self.probe.set_framing(Some(format))
    }

    fn flush(&mut self) -> Result<(), DebugProbeError> {
        if !self.steps.is_empty() {
            self.probe.cjtag_pins(&self.steps)?;
            self.steps.clear();
        }
        Ok(())
    }
}

/// Activates two-pin JTAG from any state, in the first of [`SWITCH_TO`] that both the probe and
/// the target have and in OScan1 otherwise, and returns the format. Framing is left on and the
/// TAP in Run-Test/Idle with BYPASS in IR.
///
/// After this, the target must not reach Test-Logic-Reset until the session ends: that drops it
/// back to JScan.
pub(crate) fn activate_two_pin(probe: &mut dyn CjtagProbe) -> Result<ScanFormat, DebugProbeError> {
    probe.set_framing(None)?;
    let offered = probe.scan_formats()?;
    let candidates: Vec<_> = SWITCH_TO
        .into_iter()
        .filter(|format| offered.contains(format))
        .collect();

    let mut wire = Wire::new(probe);
    wire.escape(RESET_ESCAPE_EDGES);
    wire.clock(&[true; WAKE_CLOCKS])?;
    wire.clock(&[false])?;
    wire.wait(POWER_UP);
    for step in ti_steps() {
        match step {
            Step::Jscan(tms) => wire.clock(&tms)?,
            Step::Csm(clocks) => wire.clock(&vec![false; clocks])?,
            Step::Frames(tms) => _ = wire.frames(&tms)?,
        }
    }

    // A probe that frames OScan1 alone gets TI's activation unchanged.
    let format = if candidates.is_empty() {
        ScanFormat::OScan1
    } else {
        let cnfg0 = wire.read_cnfg0()?;
        tracing::debug!("TAP.7 CNFG0: {cnfg0:#010x}");
        candidates
            .into_iter()
            .find(|&format| has_format(cnfg0, format))
            .unwrap_or(ScanFormat::OScan1)
    };
    if format != ScanFormat::OScan1 {
        wire.set_format(format)?;
    }
    wire.command(STMC, ECL)?;
    wire.flush()?;
    Ok(format)
}

/// Ends a two-pin session from `from` the way a pin reset would leave the target, if
/// KeepPoweredinTLR is clear: the TAP reaches Test-Logic-Reset, which drops the format, powers the
/// JTAG domain down and clears halt-in-boot (SWCU185G 6.4, 6.6).
///
/// Only the clocks that reach the reset are sent. Eight more edges after it, now JScan clocks,
/// would wake the domain again.
pub(crate) fn power_down(
    probe: &mut dyn CjtagProbe,
    from: TapState,
) -> Result<(), DebugProbeError> {
    probe.cjtag_tms(&vec![true; clocks_to_test_logic_reset(from)])?;
    probe.set_framing(None)
}

fn clocks_to_test_logic_reset(from: TapState) -> usize {
    let mut state = FullTapState::from(from);
    let mut clocks = 0;
    while state != FullTapState::TestLogicReset {
        state = state.step(true);
        clocks += 1;
    }
    clocks
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The CNFG0 a CC1312R1 reads: class T4, revision 1, OScan2 to OScan7.
    const CC1312_CNFG0: u32 = 0x03c0_0714;

    /// What the target's TAP.7 controller takes the wire to carry.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Adapter {
        /// Whatever a previous session left, until a reset escape.
        Unknown,
        /// One TMS bit a clock.
        JScan,
        /// Frames of this format.
        OScan(ScanFormat),
    }

    /// A command whose second part reached Update-DR, waiting for the TAP's clock there.
    #[derive(Clone, Copy, Debug)]
    struct Completed {
        opcode: usize,
        operand: usize,
    }

    /// A probe wired to a model of a CC13xx's TAP.7 controller, as far as IEEE 1149.7-2022 covers
    /// what the activation sends. It panics on any clock the target would take differently from
    /// how the host meant it: a frame of the wrong format, a raw clock where the target wants a
    /// frame, a Check Packet of the wrong length.
    struct Target {
        offers: Vec<ScanFormat>,
        cnfg0: u32,
        pins: Vec<PinStep>,

        framing: Option<ScanFormat>,
        /// A raw clock since the probe last started framing outside OScan1, which it frames from
        /// a TAP state it tracks itself.
        tracking_stale: bool,

        tckc: bool,
        tmsc: bool,
        escape_edges: usize,
        escapes: Vec<usize>,

        adapter: Adapter,
        tap: Option<FullTapState>,
        /// TMS-high clocks since the escape, while the TAP state is unknown.
        ones: usize,
        shift_clocks: usize,
        /// DR scan lengths since the escape, until the command window opens.
        scans: Vec<usize>,
        window: bool,
        first_part: Option<usize>,
        completed: Option<Completed>,
        /// The raw clocks left of a Check Packet, the adapter after it, and whether it closes
        /// the window.
        check_packet: Option<(usize, Adapter, bool)>,
        cr_scan_next: bool,
        cr_scan: bool,
        commands: Vec<(usize, usize)>,
    }

    impl Target {
        fn new(offers: &[ScanFormat], cnfg0: u32) -> Self {
            Self {
                offers: offers.to_vec(),
                cnfg0,
                pins: Vec::new(),
                framing: None,
                tracking_stale: false,
                tckc: false,
                tmsc: false,
                escape_edges: 0,
                escapes: Vec::new(),
                adapter: Adapter::Unknown,
                tap: None,
                ones: 0,
                shift_clocks: 0,
                scans: Vec::new(),
                window: false,
                first_part: None,
                completed: None,
                check_packet: None,
                cr_scan_next: false,
                cr_scan: false,
                commands: Vec::new(),
            }
        }

        fn raw_clock(&mut self) {
            if let Some((left, adapter, closes)) = self.check_packet.as_mut() {
                *left -= 1;
                if *left == 0 {
                    self.adapter = *adapter;
                    self.window &= !*closes;
                    self.check_packet = None;
                }
                return;
            }
            match self.adapter {
                Adapter::Unknown => {}
                Adapter::JScan => _ = self.tap_clock(self.tmsc),
                Adapter::OScan(format) => {
                    panic!("a raw clock in {format:?}, outside a Check Packet")
                }
            }
        }

        fn frame(&mut self, tms: bool) -> bool {
            let format = self.framing.expect("a frame with framing off");
            assert!(self.check_packet.is_none(), "a frame during a Check Packet");
            assert_eq!(
                self.adapter,
                Adapter::OScan(format),
                "framed in {format:?} for a target in {:?}",
                self.adapter
            );
            assert!(
                !self.tracking_stale,
                "framed in {format:?} after a raw clock"
            );
            self.tap_clock(tms)
        }

        /// One clock the TAP sees, returning TDO.
        fn tap_clock(&mut self, tms: bool) -> bool {
            let Some(state) = self.tap else {
                self.ones = if tms { self.ones + 1 } else { 0 };
                if self.ones == 5 {
                    self.tap = Some(FullTapState::TestLogicReset);
                }
                return true;
            };
            if let Some(completed) = self.completed.take() {
                self.schedule_check_packet(completed);
            }
            let mut tdo = true;
            if state == FullTapState::ShiftDr {
                if self.cr_scan {
                    tdo = self.cnfg0 >> self.shift_clocks & 1 != 0;
                }
                self.shift_clocks += 1;
            }
            let next = state.step(tms);
            self.tap = Some(next);
            match next {
                FullTapState::CaptureDr => {
                    self.shift_clocks = 0;
                    self.cr_scan = std::mem::take(&mut self.cr_scan_next);
                }
                FullTapState::UpdateDr => self.dr_scan(self.shift_clocks),
                _ => {}
            }
            tdo
        }

        fn dr_scan(&mut self, clocks: usize) {
            if !self.window {
                self.scans.push(clocks);
                self.window = self.scans.ends_with(&[0, 0, 1]);
                if self.window {
                    self.scans.clear();
                }
                return;
            }
            if std::mem::take(&mut self.cr_scan) {
                return;
            }
            let Some(opcode) = self.first_part.take() else {
                self.first_part = Some(clocks);
                return;
            };
            self.commands.push((opcode, clocks));
            self.cr_scan_next = opcode == SCNS;
            self.completed = Some(Completed {
                opcode,
                operand: clocks,
            });
        }

        /// The Check Packet that follows the clock in a command's Update-DR (IEEE 1149.7-2022
        /// Table 21-5): after STFMT alone in JScan, after a scan packet the TAP does not act on
        /// in OScan.
        fn schedule_check_packet(&mut self, Completed { opcode, operand }: Completed) {
            let next = if opcode == STFMT {
                Adapter::OScan(match operand {
                    9 => ScanFormat::OScan1,
                    10 => ScanFormat::OScan2,
                    13 => ScanFormat::OScan5,
                    14 => ScanFormat::OScan6,
                    other => panic!("STFMT {other}"),
                })
            } else {
                self.adapter
            };
            let closes = opcode == STMC && operand == ECL;
            let clocks = match self.adapter {
                Adapter::JScan if opcode == STFMT => CHECK_PACKET_CLOCKS,
                Adapter::JScan => return,
                Adapter::OScan(format) => non_shift_payload_bits(format) + CHECK_PACKET_CLOCKS,
                Adapter::Unknown => unreachable!(),
            };
            self.check_packet = Some((clocks, next, closes));
        }
    }

    /// The bits of a scan packet outside Shift-xR, counted from IEEE 1149.7-2022 Table 25-1.
    fn non_shift_payload_bits(format: ScanFormat) -> usize {
        match format {
            // nTDI, TMS and TDO.
            ScanFormat::OScan1 | ScanFormat::OScan5 => 3,
            // TMS.
            ScanFormat::OScan2 | ScanFormat::OScan6 => 1,
        }
    }

    impl CjtagProbe for Target {
        fn cjtag_pins(&mut self, steps: &[PinStep]) -> Result<(), DebugProbeError> {
            self.pins.extend_from_slice(steps);
            for &step in steps {
                match step {
                    PinStep::Tckc(level) => {
                        if self.tckc && !level && self.escape_edges > 0 {
                            assert!(
                                self.escape_edges >= 8,
                                "an escape of {} edges",
                                self.escape_edges
                            );
                            self.escapes.push(self.escape_edges);
                            self.escape_edges = 0;
                            self.adapter = Adapter::JScan;
                            self.tap = None;
                            self.ones = 0;
                            self.scans.clear();
                            self.window = false;
                            self.first_part = None;
                            self.completed = None;
                            self.check_packet = None;
                        }
                        if !self.tckc && level {
                            self.raw_clock();
                            if self.framing.is_some_and(|f| f != ScanFormat::OScan1) {
                                self.tracking_stale = true;
                            }
                        }
                        self.tckc = level;
                    }
                    PinStep::Tmsc(level) => {
                        if self.tckc && level != self.tmsc {
                            self.escape_edges += 1;
                        }
                        self.tmsc = level;
                    }
                    PinStep::Wait(_) => {}
                }
            }
            Ok(())
        }

        fn scan_formats(&mut self) -> Result<Vec<ScanFormat>, DebugProbeError> {
            Ok(self.offers.clone())
        }

        fn set_framing(&mut self, format: Option<ScanFormat>) -> Result<(), DebugProbeError> {
            if let Some(format) = format.filter(|&f| f != ScanFormat::OScan1) {
                assert!(self.offers.contains(&format), "{format:?} is not offered");
                assert_eq!(
                    self.tap,
                    Some(FullTapState::RunTestIdle),
                    "{format:?} framing started outside Run-Test/Idle"
                );
            }
            self.framing = format;
            self.tracking_stale = false;
            Ok(())
        }

        fn cjtag_tms(&mut self, tms: &[bool]) -> Result<Vec<bool>, DebugProbeError> {
            let tdo = tms.iter().map(|&bit| self.frame(bit)).collect();
            // Where the frames leave the pins.
            self.tckc = false;
            self.tmsc = tms.last().copied().unwrap_or(self.tmsc);
            Ok(tdo)
        }
    }

    /// The commands TI's routine sends before ECL.
    const TI_COMMANDS: [(usize, usize); 4] = [(1, 1), (STFMT, 9), (STMC, 4), (7, 0)];

    fn activate(target: &mut Target) -> ScanFormat {
        let format = activate_two_pin(target).unwrap();
        assert_eq!(target.adapter, Adapter::OScan(format));
        assert_eq!(target.framing, Some(format));
        assert_eq!(target.tap, Some(FullTapState::RunTestIdle));
        assert!(!target.window, "the command window is still open");
        format
    }

    #[test]
    fn a_probe_that_frames_only_oscan1_gets_tis_activation() {
        let mut target = Target::new(&[ScanFormat::OScan1], CC1312_CNFG0);
        assert_eq!(activate(&mut target), ScanFormat::OScan1);
        assert_eq!(
            target.commands,
            [&TI_COMMANDS[..], &[(STMC, ECL)][..]].concat()
        );
    }

    #[test]
    fn a_target_with_oscan2_is_switched_to_it() {
        let mut target = Target::new(&[ScanFormat::OScan1, ScanFormat::OScan2], CC1312_CNFG0);
        assert_eq!(activate(&mut target), ScanFormat::OScan2);
        assert_eq!(
            target.commands,
            [
                &TI_COMMANDS[..],
                &[(MCM, 0), (SCNS, CNFG0), (STFMT, 10), (STMC, ECL)][..]
            ]
            .concat()
        );
    }

    #[test]
    fn a_probe_without_oscan2_switches_to_oscan5() {
        let mut target = Target::new(&[ScanFormat::OScan1, ScanFormat::OScan5], CC1312_CNFG0);
        assert_eq!(activate(&mut target), ScanFormat::OScan5);
        assert_eq!(
            target.commands,
            [
                &TI_COMMANDS[..],
                &[(MCM, 0), (SCNS, CNFG0), (STFMT, 13), (STMC, ECL)][..]
            ]
            .concat()
        );
    }

    #[test]
    fn a_probe_with_only_oscan6_switches_to_it() {
        let mut target = Target::new(&[ScanFormat::OScan1, ScanFormat::OScan6], CC1312_CNFG0);
        assert_eq!(activate(&mut target), ScanFormat::OScan6);
        assert_eq!(
            target.commands,
            [
                &TI_COMMANDS[..],
                &[(MCM, 0), (SCNS, CNFG0), (STFMT, 14), (STMC, ECL)][..]
            ]
            .concat()
        );
    }

    #[test]
    fn the_formats_are_tried_in_order() {
        let all = [
            ScanFormat::OScan1,
            ScanFormat::OScan2,
            ScanFormat::OScan5,
            ScanFormat::OScan6,
        ];
        let mut target = Target::new(&all, CC1312_CNFG0);
        assert_eq!(activate(&mut target), ScanFormat::OScan6);

        // A target without the OScan7:6 pair.
        let mut target = Target::new(&all, CC1312_CNFG0 & !(1 << 24));
        assert_eq!(activate(&mut target), ScanFormat::OScan5);

        // A target with the OScan3:2 pair alone.
        let mut target = Target::new(&all, CC1312_CNFG0 & !(1 << 24) & !(1 << 23));
        assert_eq!(activate(&mut target), ScanFormat::OScan2);
    }

    #[test]
    fn a_read_nothing_drove_stays_in_oscan1() {
        let mut target = Target::new(&[ScanFormat::OScan1, ScanFormat::OScan2], u32::MAX);
        assert_eq!(activate(&mut target), ScanFormat::OScan1);
        assert_eq!(
            target.commands,
            [
                &TI_COMMANDS[..],
                &[(MCM, 0), (SCNS, CNFG0), (STMC, ECL)][..]
            ]
            .concat()
        );
    }

    #[test]
    fn cnfg0_says_which_formats_the_target_has() {
        for format in SWITCH_TO {
            assert!(has_format(CC1312_CNFG0, format));
            // Class T3, which has no OScan formats.
            assert!(!has_format(CC1312_CNFG0 & !0xF | 3, format));
            assert!(!has_format(0, format));
        }
        // OScan5:4 and OScan7:6 without OScan3:2.
        assert!(!has_format(CC1312_CNFG0 & !(1 << 22), ScanFormat::OScan2));
        assert!(has_format(CC1312_CNFG0 & !(1 << 22), ScanFormat::OScan5));
        // OScan3:2 and OScan7:6 without OScan5:4.
        assert!(!has_format(CC1312_CNFG0 & !(1 << 23), ScanFormat::OScan5));
        // OScan3:2 and OScan5:4 without OScan7:6.
        assert!(!has_format(CC1312_CNFG0 & !(1 << 24), ScanFormat::OScan6));
    }

    #[test]
    fn the_command_builder_writes_tis_commands() {
        // MSC 0 and ECL as TI's routine has them, from Run-Test/Idle.
        assert_eq!(
            command(7, 0),
            [
                tms(0x05, 4),
                tms(0x01, 2),
                tms(0x40, 7),
                tms(0x01, 1),
                tms(0x05, 4),
                tms(0x03, 3)
            ]
            .concat()
        );
        assert_eq!(
            command(STMC, ECL),
            [
                tms(0x05, 4),
                tms(0x03, 2),
                tms(0x05, 4),
                tms(0x01, 2),
                tms(0x01, 1),
                tms(0x01, 2)
            ]
            .concat()
        );
    }

    #[test]
    fn the_reset_takes_three_clocks_from_idle() {
        assert_eq!(clocks_to_test_logic_reset(TapState::RunTestIdle), 3);
        assert_eq!(clocks_to_test_logic_reset(TapState::ShiftDr), 5);
        assert_eq!(clocks_to_test_logic_reset(TapState::TestLogicReset), 0);
    }

    #[test]
    fn the_only_escape_is_the_reset() {
        let mut target = Target::new(&[ScanFormat::OScan1, ScanFormat::OScan2], CC1312_CNFG0);
        activate(&mut target);
        assert_eq!(target.escapes, [RESET_ESCAPE_EDGES]);
    }

    #[test]
    fn the_power_up_wait_comes_after_the_wake_clocks() {
        let mut target = Target::new(&[ScanFormat::OScan1], CC1312_CNFG0);
        activate(&mut target);

        let wait = target
            .pins
            .iter()
            .position(|step| *step == PinStep::Wait(POWER_UP))
            .unwrap();
        let rising = target.pins[..wait]
            .iter()
            .filter(|step| **step == PinStep::Tckc(true))
            .count();
        // The first write raises the clock, the escape needs it high, then the wake clocks and
        // one idle clock.
        assert!(rising > WAKE_CLOCKS);
    }

    #[test]
    fn power_down_reaches_test_logic_reset_in_frames() {
        let mut target = Target::new(&[ScanFormat::OScan1, ScanFormat::OScan2], CC1312_CNFG0);
        activate(&mut target);
        power_down(&mut target, TapState::RunTestIdle).unwrap();
        assert_eq!(target.tap, Some(FullTapState::TestLogicReset));
        assert_eq!(target.framing, None);
    }
}
