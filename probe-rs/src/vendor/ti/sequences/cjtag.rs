//! Activating two-pin OScan1 on a TI CC13xx/CC26xx (SWCU185G 6.2 to 6.4).
//!
//! This is the routine TI's XDS110 firmware runs, as a TI engineer posted it in TI E2E thread
//! 766675 ("CC1352P: How to enable cJTAG OSCAN1 mode on CC13xx and CC26xx device"), with two
//! additions. TI's routine starts from a powered-down JTAG domain and leaves out the power-up
//! wait the TRM requires. Here a reset escape and more wake clocks come first, so it also
//! attaches to a target a previous session left in OScan1, or half woken.
//!
//! The adapter powers up in two-pin JScan, where TMSC is TMS on every clock. Command scans then
//! select OScan1, and from there every JTAG clock is a three-packet frame. Between some steps the
//! adapter's control state machine (CSM) consumes clocks that the TAP never sees.

use std::time::Duration;

use crate::probe::{CjtagProbe, DebugProbeError, PinStep, TapState, jtag::FullTapState};

/// TMSC edges with TCKC high that reset the adapter, whatever format it is in: 8 or more is a
/// reset escape. The J-Link sends 10.
const RESET_ESCAPE_EDGES: usize = 10;

/// TMS-high clocks that wake the JTAG power domain. ICEMelter wants 8 rising and 8 falling edges
/// within 4 ms (TRM 6.4); a domain left half woken needs more than 8.
const WAKE_CLOCKS: usize = 20;

/// The JTAG power domain's power-up time before it takes commands (TRM 6.4).
const POWER_UP: Duration = Duration::from_micros(200);

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

/// TI's routine after its Test-Logic-Reset, from Run-Test/Idle. Every DR scan's Shift-DR clock
/// count is a command part, so one clock more or less is a different command.
fn ti_oscan1_steps() -> Vec<Step> {
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
        // CCE 0: select controller 0.
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
        // ECL: close the command window, which couples the TAP again. Ends in Run-Test/Idle.
        Step::Frames(tms(0x05, 4)),
        Step::Frames(
            [
                tms(0x03, 2),
                tms(0x05, 4),
                tms(0x01, 2),
                tms(0x01, 1),
                tms(0x01, 2),
            ]
            .concat(),
        ),
        Step::Csm(7),
    ]
}

/// The pins as the activation drives them, tracking their levels so that TMSC moves only while
/// TCKC is low, except in an escape.
struct Wire<'a> {
    probe: &'a mut dyn CjtagProbe,
    steps: Vec<PinStep>,
    tckc: bool,
    tmsc: bool,
    framing: bool,
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
            framing: false,
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
        if self.framing {
            self.start_framing()?;
        }
        Ok(())
    }

    fn wait(&mut self, wait: Duration) {
        self.steps.push(PinStep::Wait(wait));
    }

    fn frames(&mut self, tms: &[bool]) -> Result<(), DebugProbeError> {
        if !self.framing {
            self.start_framing()?;
            self.framing = true;
        }
        self.probe.cjtag_tms(tms)?;
        // A frame ends with TCKC low and TMSC back at its TMS bit.
        self.tckc = false;
        self.tmsc = tms.last().copied().unwrap_or(self.tmsc);
        Ok(())
    }

    /// Every frame starts with TCKC low.
    fn start_framing(&mut self) -> Result<(), DebugProbeError> {
        self.steps.push(PinStep::Tckc(false));
        self.tckc = false;
        self.flush()?;
        self.probe.set_oscan1_framing(true)
    }

    fn flush(&mut self) -> Result<(), DebugProbeError> {
        if !self.steps.is_empty() {
            self.probe.cjtag_pins(&self.steps)?;
            self.steps.clear();
        }
        Ok(())
    }
}

/// Activates OScan1 from any state, leaving framing on and the TAP in Run-Test/Idle with BYPASS
/// in IR.
///
/// After this, the target must not reach Test-Logic-Reset until the session ends: that drops it
/// back to JScan.
pub(crate) fn activate_oscan1(probe: &mut dyn CjtagProbe) -> Result<(), DebugProbeError> {
    probe.set_oscan1_framing(false)?;

    let mut wire = Wire::new(probe);
    wire.escape(RESET_ESCAPE_EDGES);
    wire.clock(&[true; WAKE_CLOCKS])?;
    wire.clock(&[false])?;
    wire.wait(POWER_UP);
    for step in ti_oscan1_steps() {
        match step {
            Step::Jscan(tms) => wire.clock(&tms)?,
            Step::Csm(clocks) => wire.clock(&vec![false; clocks])?,
            Step::Frames(tms) => wire.frames(&tms)?,
        }
    }
    wire.flush()
}

/// Ends a two-pin session from `from` the way a pin reset would leave the target, if
/// KeepPoweredinTLR is clear: the TAP reaches Test-Logic-Reset, which drops OScan1, powers the
/// JTAG domain down and clears halt-in-boot (SWCU185G 6.4, 6.6).
///
/// Only the clocks that reach the reset are sent. Eight more edges after it, now JScan clocks,
/// would wake the domain again.
pub(crate) fn power_down(
    probe: &mut dyn CjtagProbe,
    from: TapState,
) -> Result<(), DebugProbeError> {
    probe.cjtag_tms(&vec![true; clocks_to_test_logic_reset(from)])?;
    probe.set_oscan1_framing(false)
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

    #[test]
    fn the_reset_takes_three_clocks_from_idle() {
        assert_eq!(clocks_to_test_logic_reset(TapState::RunTestIdle), 3);
        assert_eq!(clocks_to_test_logic_reset(TapState::ShiftDr), 5);
        assert_eq!(clocks_to_test_logic_reset(TapState::TestLogicReset), 0);
    }

    /// The Shift-DR clocks of each DR scan the TAP sees, from Run-Test/Idle.
    fn dr_scans(steps: &[Step]) -> Vec<usize> {
        let mut state = FullTapState::RunTestIdle;
        let mut scans = Vec::new();
        let mut shift = None;
        for step in steps {
            let (Step::Jscan(tms) | Step::Frames(tms)) = step else {
                continue;
            };
            for &bit in tms {
                if state == FullTapState::ShiftDr {
                    *shift.as_mut().unwrap() += 1;
                }
                state = state.step(bit);
                match state {
                    FullTapState::CaptureDr => shift = Some(0),
                    FullTapState::UpdateDr => scans.push(shift.take().unwrap()),
                    _ => {}
                }
            }
        }
        assert_eq!(state, FullTapState::RunTestIdle);
        scans
    }

    #[test]
    fn the_scans_are_the_documented_commands() {
        assert_eq!(
            dr_scans(&ti_oscan1_steps()),
            [
                0, 0, 1, // open the window and lock it
                1, 1, // STC1 1
                3, 9, // STFMT 9
                0, 4, // STMC 4
                7, 0, // CCE 0
                0, 1, // ECL
            ]
        );
    }

    /// Records what the activation asks of the probe.
    #[derive(Default)]
    struct Recorder {
        pins: Vec<PinStep>,
        /// For each framed clock, how many pin steps came before it.
        frames_at: Vec<usize>,
        framing: bool,
    }

    impl CjtagProbe for Recorder {
        fn cjtag_pins(&mut self, steps: &[PinStep]) -> Result<(), DebugProbeError> {
            self.pins.extend_from_slice(steps);
            Ok(())
        }

        fn set_oscan1_framing(&mut self, on: bool) -> Result<(), DebugProbeError> {
            self.framing = on;
            Ok(())
        }

        fn cjtag_tms(&mut self, tms: &[bool]) -> Result<(), DebugProbeError> {
            assert!(self.framing, "frames sent with framing off");
            self.frames_at.extend(tms.iter().map(|_| self.pins.len()));
            Ok(())
        }
    }

    #[test]
    fn tmsc_moves_with_tckc_high_only_in_the_escape() {
        let mut probe = Recorder::default();
        activate_oscan1(&mut probe).unwrap();

        // Before the first clock write the level is whatever the last session left.
        let mut tckc = false;
        let mut edges_while_high = Vec::new();
        let mut run = 0;
        for step in &probe.pins {
            match step {
                PinStep::Tckc(level) => {
                    if tckc && !level && run > 0 {
                        edges_while_high.push(run);
                    }
                    if !level {
                        run = 0;
                    }
                    tckc = *level;
                }
                PinStep::Tmsc(_) if tckc => run += 1,
                _ => {}
            }
        }
        assert_eq!(edges_while_high, [RESET_ESCAPE_EDGES]);
        assert!(probe.framing);
        assert_eq!(probe.frames_at.len(), 3 + 14 + 4 + 17 + 4 + 11);
    }

    #[test]
    fn the_power_up_wait_comes_after_the_wake_clocks() {
        let mut probe = Recorder::default();
        activate_oscan1(&mut probe).unwrap();

        let wait = probe
            .pins
            .iter()
            .position(|step| *step == PinStep::Wait(POWER_UP))
            .unwrap();
        let rising = probe.pins[..wait]
            .iter()
            .filter(|step| **step == PinStep::Tckc(true))
            .count();
        // The first write raises the clock, the escape needs it high, then the wake clocks and
        // one idle clock.
        assert!(rising > WAKE_CLOCKS);
    }
}
