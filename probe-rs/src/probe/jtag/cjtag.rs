//! IEEE 1149.7 two-pin JTAG (cJTAG): what a probe offers a target's activation sequence.
//!
//! The probe frames; the target's debug sequence activates. Activation is vendor specific: a TI
//! CC13xx starts in two-pin JScan and is switched to an OScan format by command scans, with clocks
//! between them that the target's adapter consumes and the TAP never sees. So the sequence drives
//! the pins itself up to that point, then turns framing on.

use std::time::Duration;

use crate::probe::DebugProbeError;

/// One step of a pin program on TCKC and TMSC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinStep {
    /// Drive TCKC to this level.
    Tckc(bool),
    /// Drive TMSC to this level.
    Tmsc(bool),
    /// Wait before the next step.
    Wait(Duration),
}

/// An IEEE 1149.7 scan format, the shape of the frame one JTAG clock takes on the two pins.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScanFormat {
    /// nTDI, TMS and TDO on every clock.
    OScan1,
    /// TMS alone outside Shift-xR; nTDI, TMS and TDO in it.
    OScan2,
}

impl ScanFormat {
    /// The format's number, as the STFMT command and the SCNFMT register give it.
    pub fn number(self) -> u8 {
        match self {
            ScanFormat::OScan1 => 9,
            ScanFormat::OScan2 => 10,
        }
    }
}

/// A probe that drives an IEEE 1149.7 target over two pins, TCKC and TMSC.
///
/// With framing on, every JTAG clock the probe sends, batched or not, leaves as one frame of the
/// format it was turned on in.
pub trait CjtagProbe {
    /// Drives the pins step by step, unframed, with nothing between the steps but the waits.
    ///
    /// TMSC moving while TCKC is high is an escape edge to the target, so a program moves it with
    /// TCKC low except where it means to escape.
    fn cjtag_pins(&mut self, steps: &[PinStep]) -> Result<(), DebugProbeError>;

    /// The formats the probe can frame.
    fn scan_formats(&mut self) -> Result<Vec<ScanFormat>, DebugProbeError>;

    /// Frames every JTAG clock in `format` from here on, or stops framing.
    ///
    /// The target must already be in `format`. Outside OScan1 a frame's shape depends on the TAP
    /// state, which the probe tracks from the TMS it sends, starting from Run-Test/Idle. So for any
    /// format but OScan1 the TAP must be in Run-Test/Idle when this is called, and a clock from
    /// [`Self::cjtag_pins`] leaves the tracking wrong until the next call.
    ///
    /// Turning framing on leaves TCKC low and TMSC driven, the state every frame starts from.
    /// Turning it off first completes a TAP advance the format left pending (IEEE 1149.7-2022
    /// 25.6), where the TAP is in Test-Logic-Reset or Run-Test/Idle and one more clock of the
    /// same TMS cannot move it.
    fn set_framing(&mut self, format: Option<ScanFormat>) -> Result<(), DebugProbeError>;

    /// Sends one framed JTAG clock for each bit of `tms`, with TDI high, and returns TDO for each.
    ///
    /// The JTAG chain's TAP state does not follow these clocks; the caller records where they
    /// leave the TAP.
    fn cjtag_tms(&mut self, tms: &[bool]) -> Result<Vec<bool>, DebugProbeError>;
}
