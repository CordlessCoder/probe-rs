//! Two-pin JTAG (IEEE 1149.7) through a CMSIS-DAP probe with cJTAG framing support.
//!
//! With framing on, the firmware sends every JTAG clock of `DAP_JTAG_Sequence` and
//! `DAP_Transfer` as one frame of the format it was turned on in. Activating the format is the
//! target's business: its debug sequence drives the pins through [`CjtagProbe::cjtag_pins`], then
//! turns framing on.
//!
//! The pins are written one `DAP_SWJ_Pins` at a time, batched in `DAP_ExecuteCommands`, rather
//! than with `DAP_SWJ_Sequence`: that moves TMS while TCK is high, which a two-pin target reads as
//! an escape edge.

use bitvec::prelude::*;

use super::{
    CmsisDap, CmsisDapError,
    commands::{
        self,
        general::execute::{PinCommand, PinProgramRequest},
        jtag::sequence::{Sequence as JtagSequence, SequenceRequest as JtagSequenceRequest},
    },
};
use crate::probe::{CjtagProbe, DebugProbeError, PinStep, Pins, ScanFormat};

/// TCKC is SWCLK/TCK, bit 0 of `DAP_SWJ_Pins`.
const TCKC: u8 = 1 << 0;
/// TMSC is SWDIO/TMS, bit 1.
const TMSC: u8 = 1 << 1;

/// Clocks one `DAP_JTAG_Sequence` sequence carries.
const MAX_SEQUENCE_CLOCKS: usize = 64;

impl CmsisDap {
    pub(crate) fn frames_cjtag(&mut self) -> bool {
        self.cjtag_scan_formats()
            .is_ok_and(|formats| formats.contains(&ScanFormat::OScan1))
    }

    pub(crate) fn set_cjtag_framing(
        &mut self,
        format: Option<ScanFormat>,
    ) -> Result<(), DebugProbeError> {
        self.cjtag_set_framing(format)
    }

    fn cjtag_scan_formats(&mut self) -> Result<Vec<ScanFormat>, DebugProbeError> {
        let Some(extension) = self.cjtag_extension.as_mut() else {
            return Err(DebugProbeError::NotImplemented {
                function_name: "CMSIS-DAP cJTAG vendor extension",
            });
        };
        let device = &mut self.device;
        let mut send_vendor_command = |command_id: u8, payload: &[u8]| {
            super::send_vendor_command(device, command_id, payload)
        };
        extension.scan_formats(&mut send_vendor_command)
    }

    fn cjtag_set_framing(&mut self, format: Option<ScanFormat>) -> Result<(), DebugProbeError> {
        let Some(extension) = self.cjtag_extension.as_mut() else {
            return if format.is_none() {
                Ok(())
            } else {
                Err(DebugProbeError::NotImplemented {
                    function_name: "CMSIS-DAP cJTAG vendor extension",
                })
            };
        };
        let device = &mut self.device;
        let mut send_vendor_command = |command_id: u8, payload: &[u8]| {
            super::send_vendor_command(device, command_id, payload)
        };
        extension.set_framing(format, &mut send_vendor_command)
    }

    /// The bytes one request's payload may take.
    fn request_budget(&self) -> usize {
        self.packet_size as usize - 1
    }
}

/// The `DAP_SWJ_Pins` writes and `DAP_Delay`s that carry `steps`.
fn pin_commands(steps: &[PinStep]) -> Vec<PinCommand> {
    let mut commands = Vec::with_capacity(steps.len());
    for step in steps {
        match *step {
            PinStep::Tckc(level) => commands.push(PinCommand::Write(
                Pins(if level { TCKC } else { 0 }),
                Pins(TCKC),
            )),
            PinStep::Tmsc(level) => commands.push(PinCommand::Write(
                Pins(if level { TMSC } else { 0 }),
                Pins(TMSC),
            )),
            PinStep::Wait(wait) => {
                // A zero delay hangs ARM's firmware, so round up to a microsecond.
                let mut us = wait.as_micros().max(1);
                while us > 0 {
                    let chunk = us.min(u128::from(u16::MAX));
                    commands.push(PinCommand::Delay(chunk as u16));
                    us -= chunk;
                }
            }
        }
    }
    commands
}

/// Splits `commands` into programs that each fit `budget` bytes and the one-byte count.
fn pin_programs(commands: Vec<PinCommand>, budget: usize) -> Vec<Vec<PinCommand>> {
    let mut programs = Vec::new();
    let mut program = Vec::new();
    let mut len = PinProgramRequest::HEADER_LEN;
    for command in commands {
        if !program.is_empty()
            && (len + command.len() > budget || program.len() == u8::MAX as usize)
        {
            programs.push(std::mem::take(&mut program));
            len = PinProgramRequest::HEADER_LEN;
        }
        len += command.len();
        program.push(command);
    }
    if !program.is_empty() {
        programs.push(program);
    }
    programs
}

/// `DAP_JTAG_Sequence` requests that clock `tms` with TDI high and capture TDO, each fitting
/// `budget` bytes.
fn tms_requests(tms: &[bool], budget: usize) -> Result<Vec<JtagSequenceRequest>, CmsisDapError> {
    let mut requests = Vec::new();
    let mut sequences = Vec::new();
    // The sequence count, then each sequence's info byte and TDI bytes. The reply, a status and
    // the TDO bytes, is shorter, so it fits too.
    let mut len = 1;
    for run in tms.chunk_by(|a, b| a == b) {
        for chunk in run.chunks(MAX_SEQUENCE_CLOCKS) {
            let size = 1 + chunk.len().div_ceil(8);
            if !sequences.is_empty() && (len + size > budget || sequences.len() == u8::MAX as usize)
            {
                requests.push(JtagSequenceRequest::new(std::mem::take(&mut sequences))?);
                len = 1;
            }
            let tdi = BitVec::<usize, Lsb0>::repeat(true, chunk.len());
            sequences.push(JtagSequence::capture(chunk[0], &tdi)?);
            len += size;
        }
    }
    if !sequences.is_empty() {
        requests.push(JtagSequenceRequest::new(sequences)?);
    }
    Ok(requests)
}

impl CjtagProbe for CmsisDap {
    fn cjtag_pins(&mut self, steps: &[PinStep]) -> Result<(), DebugProbeError> {
        for program in pin_programs(pin_commands(steps), self.request_budget()) {
            let request = PinProgramRequest::new(program)?;
            commands::send_command(&mut self.device, &request)?;
        }
        Ok(())
    }

    fn scan_formats(&mut self) -> Result<Vec<ScanFormat>, DebugProbeError> {
        self.cjtag_scan_formats()
    }

    fn set_framing(&mut self, format: Option<ScanFormat>) -> Result<(), DebugProbeError> {
        self.cjtag_set_framing(format)
    }

    fn cjtag_tms(&mut self, tms: &[bool]) -> Result<Vec<bool>, DebugProbeError> {
        let mut tdo = Vec::with_capacity(tms.len());
        for request in tms_requests(tms, self.request_budget())? {
            tdo.extend(self.send_jtag_sequences(request)?);
        }
        Ok(tdo)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;

    #[test]
    fn a_wait_longer_than_a_delay_command_is_split() {
        let commands = pin_commands(&[PinStep::Wait(Duration::from_micros(70_000))]);
        assert_eq!(
            commands,
            [PinCommand::Delay(u16::MAX), PinCommand::Delay(4_465)]
        );
    }

    #[test]
    fn a_wait_shorter_than_a_microsecond_still_waits() {
        assert_eq!(
            pin_commands(&[PinStep::Wait(Duration::from_nanos(10))]),
            [PinCommand::Delay(1)]
        );
    }

    #[test]
    fn programs_fit_the_packet() {
        let commands = pin_commands(&[PinStep::Tckc(true); 20]);
        // A 64-byte packet leaves 63 for the payload: the count and eight 7-byte writes.
        let programs = pin_programs(commands, 63);
        assert_eq!(programs.iter().map(Vec::len).collect::<Vec<_>>(), [8, 8, 4]);
    }

    #[test]
    fn tms_runs_become_sequences_of_at_most_64_clocks() {
        let tms: Vec<bool> = [vec![true; 70], vec![false; 3]].concat();
        let requests = tms_requests(&tms, 511).unwrap();
        assert_eq!(requests.len(), 1);
        assert_eq!(
            format!("{:?}", requests[0]).matches("tck_cycles").count(),
            3
        );
    }
}
