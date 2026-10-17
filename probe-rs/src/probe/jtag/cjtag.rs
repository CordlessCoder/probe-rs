//! IEEE 1149.7 two-pin JTAG (cJTAG): what a probe offers a target's activation sequence.
//!
//! The probe frames; the target's debug sequence activates. Activation is vendor specific: a TI
//! CC13xx starts in two-pin JScan and is switched to OScan1 by command scans, with clocks between
//! them that the target's adapter consumes and the TAP never sees. So the sequence drives the pins
//! itself up to that point, then turns framing on.

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

/// A probe that drives an IEEE 1149.7 target over two pins, TCKC and TMSC.
///
/// With OScan1 framing on, every JTAG clock the probe sends, batched or not, leaves as one frame:
/// nTDI, TMS, and a TDO packet the target drives.
pub trait CjtagProbe {
    /// Drives the pins step by step, unframed, with nothing between the steps but the waits.
    ///
    /// TMSC moving while TCKC is high is an escape edge to the target, so a program moves it with
    /// TCKC low except where it means to escape.
    fn cjtag_pins(&mut self, steps: &[PinStep]) -> Result<(), DebugProbeError>;

    /// Turns OScan1 framing on or off.
    ///
    /// Turning it on leaves TCKC low and TMSC driven, the state every frame starts from.
    fn set_oscan1_framing(&mut self, on: bool) -> Result<(), DebugProbeError>;

    /// Sends one framed JTAG clock for each bit of `tms`, with TDI high, capturing nothing.
    ///
    /// The probe's tracked TAP state does not follow these clocks; the caller records where they
    /// leave the TAP.
    fn cjtag_tms(&mut self, tms: &[bool]) -> Result<(), DebugProbeError>;
}
