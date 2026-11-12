//! Two-pin cJTAG (IEEE 1149.7) through the commands proposed for CMSIS-DAP in
//! ARM-software/CMSIS-DAP#8.
//!
//! The probe frames, the host activates. `DAP_JTAG_Format`, sent just before `DAP_Connect`, selects
//! the scan format the connection comes up in, and the target is activated with escapes, raw clocks
//! and framed scans. A walk into
//! Test-Logic-Reset takes the target back to the standard protocol, so it is activated again after
//! each one, as the J-Link backend does. The activation leaves the TAP in Run-Test/Idle, with the
//! IDCODE selected as Test-Logic-Reset would, so the clocks that would keep the TAP in
//! Test-Logic-Reset are dropped until it leaves it.

use std::time::Duration;

use bitvec::prelude::*;

use super::{
    CmsisDap, CmsisDapError,
    commands::{
        self, RequestError, Status,
        cjtag::{
            BatchCommand, BatchRequest, EscapeRequest, FormatRequest, VERGE_VENDOR,
            VergeInfoRequest,
        },
        general::{
            connect::{ConnectRequest, ConnectResponse},
            disconnect::DisconnectRequest,
            info::VendorCommand,
        },
        jtag::sequence::{Sequence as JtagSequence, SequenceRequest as JtagSequenceRequest},
        swj::sequence::SequenceRequest,
    },
};
use crate::probe::{
    DebugProbeError,
    jtag::{FullTapState, TapState},
};

/// An IEEE 1149.7 scan format: the shape one JTAG clock takes on the two pins.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ScanFormat {
    /// nTDI, TMS and TDO on every clock.
    #[default]
    OScan1,
    /// TMS alone outside Shift-xR; nTDI, TMS and TDO in it.
    OScan2,
    /// nTDI, TMS and TDO outside Shift-xR; nTDI and TDO in it.
    OScan5,
    /// TMS alone outside Shift-xR; nTDI and TDO in it.
    OScan6,
}

impl ScanFormat {
    /// Every format the commands can select.
    pub const ALL: [ScanFormat; 4] = [Self::OScan1, Self::OScan2, Self::OScan5, Self::OScan6];

    /// The OScan number, which is also the value `DAP_JTAG_Format` selects it with.
    pub fn oscan(self) -> u8 {
        match self {
            Self::OScan1 => 1,
            Self::OScan2 => 2,
            Self::OScan5 => 5,
            Self::OScan6 => 6,
        }
    }

    /// The value the STFMT command stores in the controller's SCNFMT register.
    fn scnfmt(self) -> usize {
        usize::from(self.oscan()) + 8
    }
}

/// The time a CC1312 needed its TCKC clocked with TMSC high to come out of a reset escape, from a
/// live session and from a pin reset alike.
const WAKE: Duration = Duration::from_nanos(6_250);
/// The fewest wake clocks, whatever the clock rate.
const MIN_WAKE_CLOCKS: usize = 22;
/// The wait after the wake for a target's debug logic to power up: TI's 200 µs (SWCU185G 6.4).
const POWER_UP: Duration = Duration::from_micros(200);
/// Bits one `DAP_SWJ_Sequence` carries.
const MAX_RAW_BITS: usize = 256;
/// Clocks one `DAP_JTAG_Sequence` sequence carries.
const MAX_SEQUENCE_CLOCKS: usize = 64;

/// The two-pin state of a probe that frames cJTAG.
#[derive(Debug)]
pub(super) struct Cjtag {
    /// The format the next connect selects.
    format: ScanFormat,
    /// Whether the probe has the cJTAG commands, once asked.
    supported: Option<bool>,
    /// Whether the target is activated: the JTAG sequences are framed and walks into
    /// Test-Logic-Reset are watched.
    active: bool,
    /// The TAP state that the clocks sent so far lead to.
    state: FullTapState,
    /// The target was activated after a walk into Test-Logic-Reset: the TAP is in Run-Test/Idle,
    /// while the clocks sent so far lead to Test-Logic-Reset.
    parked: bool,
}

impl Cjtag {
    pub(super) fn is_active(&self) -> bool {
        self.active
    }

    pub(super) fn new() -> Self {
        Self {
            format: ScanFormat::default(),
            supported: None,
            active: false,
            state: FullTapState::TestLogicReset,
            parked: false,
        }
    }
}

impl CmsisDap {
    /// The scan formats this probe can frame. The probe has to be disconnected: it is asked by
    /// trying each format as the next connect's, which touches no pins, then clearing it with
    /// format 0. A disconnect goes first, in case a host that died left a connection, whose format
    /// the tries would change instead (and an SWD connection refuses them all).
    pub fn cjtag_scan_formats(&mut self) -> Result<Vec<ScanFormat>, DebugProbeError> {
        if self.connected {
            return Err(DebugProbeError::Other(
                "the scan formats can only be asked while disconnected".to_string(),
            ));
        }
        if !self.has_cjtag() {
            return Ok(Vec::new());
        }
        let tries = ScanFormat::ALL.map(ScanFormat::oscan);
        let mut offered = Vec::new();
        if self.capabilities.atomic_commands_implemented {
            let mut commands = vec![BatchCommand::Disconnect];
            commands.extend(tries.map(BatchCommand::Format));
            commands.push(BatchCommand::Format(0));
            let answers = commands::send_command(&mut self.device, &BatchRequest { commands })?;
            for (format, status) in ScanFormat::ALL.into_iter().zip(answers.into_iter().skip(1)) {
                if matches!(Status::from_byte(status), Ok(Status::DapOk)) {
                    offered.push(format);
                }
            }
        } else {
            commands::send_command(&mut self.device, &DisconnectRequest {})?;
            for format in ScanFormat::ALL {
                if matches!(self.select_format(format.oscan())?, Status::DapOk) {
                    offered.push(format);
                }
            }
            self.select_format(0)?;
        }
        Ok(offered)
    }

    /// Frames in `format` from the next connect with the cJTAG protocol. OScan1 is the default.
    pub fn set_cjtag_scan_format(&mut self, format: ScanFormat) {
        self.cjtag.format = format;
    }

    /// Whether the probe has the cJTAG commands.
    pub(super) fn has_cjtag(&mut self) -> bool {
        if let Some(supported) = self.cjtag.supported {
            return supported;
        }
        let supported = self.ask_cjtag();
        tracing::debug!("cJTAG commands: {supported}");
        self.cjtag.supported = Some(supported);
        supported
    }

    fn ask_cjtag(&mut self) -> bool {
        let vendor = commands::send_command(&mut self.device, &VendorCommand {});
        // The string keeps its zero terminator.
        if !matches!(vendor, Ok(Some(ref name)) if name.trim_end_matches('\0') == VERGE_VENDOR) {
            return false;
        }
        commands::send_command(&mut self.device, &VergeInfoRequest).unwrap_or(false)
    }

    fn select_format(&mut self, format: u8) -> Result<Status, CmsisDapError> {
        commands::send_command(&mut self.device, &FormatRequest { format })
    }

    /// A disconnect, the format, then `DAP_Connect` of the JTAG port, which comes up in it with the
    /// pins never conventional. The disconnect ends any connection a host that died left: on it the
    /// format would change that connection, and the connect after it would come up conventional.
    /// Where the probe has `DAP_ExecuteCommands` they go out in one, so a host that dies between
    /// them can't leave the format for the next host's connect.
    pub(super) fn cjtag_connect(&mut self) -> Result<ConnectResponse, CmsisDapError> {
        let format = self.cjtag.format.oscan();
        let refused = || CmsisDapError::ErrorResponse(RequestError::CjtagFormat { format });
        if !self.capabilities.atomic_commands_implemented {
            commands::send_command(&mut self.device, &DisconnectRequest {})?;
            if !matches!(self.select_format(format)?, Status::DapOk) {
                return Err(refused());
            }
            let response = commands::send_command(&mut self.device, &ConnectRequest::Jtag)?;
            if !matches!(response, ConnectResponse::SuccessfulInitForJTAG) {
                // A refused connect leaves the format held for the next host's.
                self.select_format(0)?;
            }
            return Ok(response);
        }
        let request = BatchRequest {
            commands: vec![
                BatchCommand::Disconnect,
                BatchCommand::Format(format),
                BatchCommand::ConnectJtag,
            ],
        };
        let answers = commands::send_command(&mut self.device, &request)?;
        if !matches!(Status::from_byte(answers[1]), Ok(Status::DapOk)) {
            // The connect ran anyway, in whatever the probe held before; the disconnect ends it and
            // clears that.
            commands::send_command(&mut self.device, &DisconnectRequest {})?;
            return Err(refused());
        }
        if answers[2] != ConnectResponse::SuccessfulInitForJTAG as u8 {
            // A refused connect leaves the format held for the next host's.
            self.select_format(0)?;
            return Ok(ConnectResponse::InitFailed);
        }
        Ok(ConnectResponse::SuccessfulInitForJTAG)
    }

    /// Selects the scan format again on the live connection.
    pub(super) fn select_cjtag_format(&mut self) -> Result<(), CmsisDapError> {
        let format = self.cjtag.format.oscan();
        match self.select_format(format)? {
            Status::DapOk => Ok(()),
            Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::CjtagFormat {
                format,
            })),
        }
    }

    /// After `DAP_Connect`: activates the target.
    pub(super) fn cjtag_after_connect(&mut self) -> Result<(), CmsisDapError> {
        self.cjtag.active = false;
        // The connect may have reset the clock, and the wake is counted from it.
        self.set_swj_clock(self.speed_khz * 1_000)?;
        self.cjtag_activate()?;
        self.cjtag.active = true;
        Ok(())
    }

    /// Before `DAP_Disconnect`. In OScan2 and OScan6 the last frame outside Shift moves the TAP only
    /// in the next bit period (IEEE 1149.7 25.6.2), and the probe sends no clock of its own, so one
    /// idle frame completes the move. The TAP rests in Run-Test/Idle, where TMS low keeps it.
    pub(super) fn cjtag_before_disconnect(&mut self) -> Result<(), CmsisDapError> {
        if self.cjtag.active && matches!(self.cjtag.format, ScanFormat::OScan2 | ScanFormat::OScan6)
        {
            self.framed(&bitvec![0; 1])?;
        }
        Ok(())
    }

    pub(super) fn cjtag_disconnected(&mut self) {
        self.cjtag.active = false;
    }

    /// The long-form activation (IEEE 1149.7 11.7) into the selected format.
    ///
    /// It leaves the TAP in Run-Test/Idle.
    fn cjtag_activate(&mut self) -> Result<(), CmsisDapError> {
        tracing::debug!("cJTAG activation into {:?}", self.cjtag.format);
        let wake = (WAKE.as_nanos() * u128::from(self.speed_khz)).div_ceil(1_000_000) as usize;

        self.escape(10, true)?;
        self.raw_clocks(&tms(&["1".repeat(wake.max(MIN_WAKE_CLOCKS)).as_str(), "0"]))?;
        std::thread::sleep(POWER_UP);
        self.escape(6, false)?;
        self.raw_clocks(&long_form_selection())?;
        self.raw_clocks(&enter_format(self.cjtag.format))?;
        // Restates Run-Test/Idle as the start of the probe's own tracking in OScan2, 5 and 6.
        self.select_cjtag_format()?;
        self.framed(&select_controller())?;
        self.raw_clocks(&tms(&["0000"]))?;
        self.framed(&tms(&[IR_PATH]))?;

        self.cjtag.state = FullTapState::TestLogicReset;
        self.cjtag.parked = true;
        Ok(())
    }

    fn escape(&mut self, edges: u8, level: bool) -> Result<(), CmsisDapError> {
        match commands::send_command(&mut self.device, &EscapeRequest { edges, level })? {
            Status::DapOk => Ok(()),
            Status::DapError => Err(CmsisDapError::ErrorResponse(RequestError::CjtagEscape {
                edges,
            })),
        }
    }

    /// TCKC clocks with TMSC at `tms`, outside any frame.
    fn raw_clocks(&mut self, tms: &BitSlice) -> Result<(), CmsisDapError> {
        for chunk in tms.chunks(MAX_RAW_BITS) {
            let mut data = vec![0u8; chunk.len().div_ceil(8)];
            for bit in chunk.iter_ones() {
                data[bit / 8] |= 1 << (bit % 8);
            }
            let bit_count = if chunk.len() == MAX_RAW_BITS {
                0
            } else {
                chunk.len() as u8
            };
            self.send_swj_sequences(SequenceRequest::new(&data, bit_count)?)?;
        }
        Ok(())
    }

    /// Framed JTAG clocks with TDI low, not watched for Test-Logic-Reset.
    fn framed(&mut self, tms: &BitSlice) -> Result<(), CmsisDapError> {
        for request in tms_requests(tms, false, self.packet_size)? {
            self.send_jtag_sequences_unwatched(request)?;
        }
        Ok(())
    }

    /// Sends `tms` as framed TMS clocks with TDI high: what a JTAG probe makes of a
    /// `DAP_SWJ_Sequence`.
    pub(super) fn cjtag_tms(&mut self, tms: &BitSlice) -> Result<(), CmsisDapError> {
        for request in tms_requests(tms, true, self.packet_size)? {
            self.send_jtag_sequences(request)?;
        }
        Ok(())
    }

    /// Sends JTAG sequences on an activated target, activating it again after each walk into
    /// Test-Logic-Reset.
    pub(super) fn cjtag_sequences(
        &mut self,
        request: JtagSequenceRequest,
    ) -> Result<BitVec, CmsisDapError> {
        // A transfer walks the TAP to Run-Test/Idle without a sequence; there the TAP already is.
        if self.cjtag.parked && self.jtag_state.tap_state == TapState::RunTestIdle {
            self.unpark();
        }

        let mut tdo = BitVec::new();
        let mut pending = Vec::new();
        for &sequence in request.sequences() {
            let tms = sequence.tms();
            let mut from = 0;
            if self.cjtag.parked {
                if !sequence.captures() {
                    if tms {
                        // Test-Logic-Reset, in effect where the TAP is.
                        continue;
                    }
                    // Run-Test/Idle, where the TAP is.
                    from = 1;
                }
                self.unpark();
            }

            let reset = (from..sequence.cycles()).find(|_| {
                self.cjtag.state = self.cjtag.state.step(tms);
                self.cjtag.state == FullTapState::TestLogicReset
            });
            let Some(reset) = reset else {
                if from < sequence.cycles() {
                    pending.push(sequence.cycles_between(from, sequence.cycles()));
                }
                continue;
            };
            pending.push(sequence.cycles_between(from, reset + 1));
            tdo.extend_from_bitslice(&self.flush(&mut pending)?);
            tracing::debug!("cJTAG: Test-Logic-Reset, activating again");
            self.cjtag_activate()?;
            if sequence.captures() && reset + 1 < sequence.cycles() {
                // Their TDO would come from a TAP moving out of Run-Test/Idle.
                return Err(CmsisDapError::ErrorResponse(
                    RequestError::CjtagCaptureInTestLogicReset,
                ));
            }
        }
        tdo.extend_from_bitslice(&self.flush(&mut pending)?);
        Ok(tdo)
    }

    fn unpark(&mut self) {
        self.cjtag.parked = false;
        self.cjtag.state = FullTapState::RunTestIdle;
    }

    fn flush(&mut self, pending: &mut Vec<JtagSequence>) -> Result<BitVec, CmsisDapError> {
        if pending.is_empty() {
            return Ok(BitVec::new());
        }
        let request = JtagSequenceRequest::new(std::mem::take(pending))?;
        self.send_jtag_sequences_unwatched(request)
    }
}

/// `DAP_JTAG_Sequence` requests that clock `tms` with TDI at `tdi`, without capturing, each fitting
/// a packet of `packet_size` bytes.
fn tms_requests(
    tms: &BitSlice,
    tdi: bool,
    packet_size: u16,
) -> Result<Vec<JtagSequenceRequest>, CmsisDapError> {
    // The command ID and the sequence count, then each sequence's info byte and TDI bytes.
    let budget = usize::from(packet_size) - 2;
    let mut requests = Vec::new();
    let mut sequences = Vec::new();
    let mut len = 0;
    for (level, clocks) in runs(tms) {
        for start in (0..clocks).step_by(MAX_SEQUENCE_CLOCKS) {
            let count = (clocks - start).min(MAX_SEQUENCE_CLOCKS);
            let size = 1 + count.div_ceil(8);
            if !sequences.is_empty() && (len + size > budget || sequences.len() == u8::MAX as usize)
            {
                requests.push(JtagSequenceRequest::new(std::mem::take(&mut sequences))?);
                len = 0;
            }
            sequences.push(JtagSequence::no_capture(
                level,
                &BitVec::<usize, Lsb0>::repeat(tdi, count),
            )?);
            len += size;
        }
    }
    if !sequences.is_empty() {
        requests.push(JtagSequenceRequest::new(sequences)?);
    }
    Ok(requests)
}

/// The runs of equal bits in `tms`, as (level, length).
fn runs(tms: &BitSlice) -> Vec<(bool, usize)> {
    let mut runs: Vec<(bool, usize)> = Vec::new();
    for bit in tms.iter().by_vals() {
        match runs.last_mut() {
            Some((level, length)) if *level == bit => *length += 1,
            _ => runs.push((bit, 1)),
        }
    }
    runs
}

/// A DR scan through `clocks` Shift-DR clocks, from Run-Test/Idle back to it, as TMS bits. A
/// controller command part is this count: one clock short is a different command.
fn dr_scan(clocks: usize) -> String {
    format!("100{}110", "0".repeat(clocks - 1))
}

/// A zero-bit scan: through Capture-DR to Update-DR without shifting.
const ZERO_BIT_SCAN: &str = "10110";

/// From Run-Test/Idle through Select-IR back to it, which leaves the command level.
const IR_PATH: &str = "110110";

fn tms(parts: &[&str]) -> BitVec {
    parts
        .iter()
        .flat_map(|part| part.chars())
        .map(|bit| bit == '1')
        .collect()
}

/// The activation code 0000 (any TAP.7), the extension code 0000 (long form, park in
/// Run-Test/Idle), 24 bits of global register state and a check packet.
fn long_form_selection() -> BitVec {
    bitvec![0; 36]
}

/// Two zero-bit scans and a one-clock scan (command level 2, locked), STC1 1, STFMT, and a check
/// packet.
fn enter_format(format: ScanFormat) -> BitVec {
    tms(&[
        ZERO_BIT_SCAN,
        ZERO_BIT_SCAN,
        &dr_scan(1),
        &dr_scan(1),
        &dr_scan(1),
        &dr_scan(3),
        &dr_scan(format.scnfmt()),
        "0000",
    ])
}

/// MSC 0, selecting controller 0, then one clock in Run-Test/Idle.
fn select_controller() -> BitVec {
    tms(&[&dr_scan(7), ZERO_BIT_SCAN, "0"])
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::probe::{
        DebugProbe, WireProtocol,
        cmsisdap::commands::{
            CmsisDapDevice,
            fake::{FakeProbe, Reply, standard_reply},
        },
    };

    /// A probe that names itself `vendor` and answers everything Verge's cJTAG commands need.
    fn probe(vendor: &'static str) -> (CmsisDap, Arc<Mutex<FakeProbe>>) {
        probe_offering(vendor, &[1, 2, 5, 6])
    }

    /// As [`probe`], framing the OScan formats in `offers`.
    fn probe_offering(
        vendor: &'static str,
        offers: &'static [u8],
    ) -> (CmsisDap, Arc<Mutex<FakeProbe>>) {
        fake(vendor, offers, true, true)
    }

    /// Like Verge, it takes a format while disconnected or on a JTAG connection, and refuses one on
    /// an SWD connection. `atomic` reports `DAP_ExecuteCommands`; without `connects`, every connect
    /// fails.
    fn fake(
        vendor: &'static str,
        offers: &'static [u8],
        atomic: bool,
        connects: bool,
    ) -> (CmsisDap, Arc<Mutex<FakeProbe>>) {
        let mut port = 0;
        let mut answer = move |command: &[u8]| -> Option<Vec<u8>> {
            Some(match command {
                [0x00, 0x01, ..] => {
                    let mut reply = vec![0x00, vendor.len() as u8 + 1];
                    reply.extend(vendor.bytes());
                    reply.push(0);
                    reply
                }
                [0x82, ..] => b"\x82CJTAG\x08".to_vec(),
                [0x02, requested, ..] => {
                    port = if connects { *requested } else { 0 };
                    vec![0x02, port]
                }
                [0x03, ..] => {
                    port = 0;
                    vec![0x03, 0x00]
                }
                [0x95, format, ..] => {
                    let takes = port != 1 && (*format == 0 || offers.contains(format));
                    vec![0x95, if takes { 0x00 } else { 0xFF }]
                }
                [id @ (0x04 | 0x11 | 0x12 | 0x14 | 0x96), ..] => vec![*id, 0x00],
                _ => return None,
            })
        };
        // SWD and JTAG, with atomic commands when asked.
        let capabilities = if atomic { 0x13 } else { 0x03 };
        let probe = FakeProbe::shared(64, move |command| match command {
            [0x00, 0xF0, ..] => Reply::Now(vec![0x00, 0x01, capabilities]),
            [0x7F, count, commands @ ..] if atomic => {
                let mut reply = vec![0x7F, *count];
                let mut rest = commands;
                for _ in 0..*count {
                    let (command, tail) = rest.split_at(if rest[0] == 0x03 { 1 } else { 2 });
                    reply.extend(answer(command).unwrap());
                    rest = tail;
                }
                Reply::Now(reply)
            }
            _ => answer(command).map_or_else(|| standard_reply(command, 64, 4), Reply::Now),
        });
        let dap = CmsisDap::new_from_device(CmsisDapDevice::Fake(probe.clone())).unwrap();
        (dap, probe)
    }

    fn attached() -> (CmsisDap, Arc<Mutex<FakeProbe>>) {
        let (mut dap, probe) = self::probe(VERGE_VENDOR);
        dap.select_protocol(WireProtocol::CJtag).unwrap();
        dap.attach().unwrap();
        (dap, probe)
    }

    fn sent(probe: &Arc<Mutex<FakeProbe>>) -> Vec<Vec<u8>> {
        probe.lock().unwrap().sent.clone()
    }

    fn sequences(tms: bool, clocks: usize) -> JtagSequenceRequest {
        let tdi = bitvec![0; clocks];
        JtagSequenceRequest::new(vec![JtagSequence::no_capture(tms, &tdi).unwrap()]).unwrap()
    }

    #[test]
    fn a_connect_comes_up_in_the_format_sent_with_it_and_activates_after_it() {
        let (_dap, probe) = attached();
        let sent = sent(&probe);
        let position = |command: &[u8]| sent.iter().position(|sent| sent == command);
        let connect = position(&[0x7F, 3, 0x03, 0x95, 1, 0x02, 0x02]).unwrap();
        let reset_escape = position(&[0x96, 10, 1]).unwrap();
        let selection_escape = position(&[0x96, 6, 0]).unwrap();
        assert!(connect < reset_escape && reset_escape < selection_escape);
    }

    #[test]
    fn without_atomic_commands_the_disconnect_and_format_go_alone_before_the_connect() {
        let (mut dap, probe) = fake(VERGE_VENDOR, &[1], false, true);
        dap.select_protocol(WireProtocol::CJtag).unwrap();
        dap.attach().unwrap();
        let sent = sent(&probe);
        let position = |command: &[u8]| sent.iter().position(|sent| sent == command).unwrap();
        assert!(position(&[0x03]) < position(&[0x95, 1]));
        assert!(position(&[0x95, 1]) < position(&[0x02, 0x02]));
    }

    #[test]
    fn a_refused_format_ends_the_connection() {
        let (mut dap, probe) = probe_offering(VERGE_VENDOR, &[1]);
        dap.set_cjtag_scan_format(ScanFormat::OScan6);
        dap.select_protocol(WireProtocol::CJtag).unwrap();
        assert!(dap.attach().is_err());
        assert_eq!(sent(&probe).last().unwrap(), &[0x03]);
    }

    #[test]
    fn a_refused_connect_clears_the_format_it_was_sent_with() {
        for atomic in [true, false] {
            let (mut dap, probe) = fake(VERGE_VENDOR, &[1], atomic, false);
            dap.select_protocol(WireProtocol::CJtag).unwrap();
            assert!(dap.attach().is_err());
            assert_eq!(sent(&probe).last().unwrap(), &[0x95, 0], "atomic: {atomic}");
        }
    }

    #[test]
    fn the_scan_formats_are_tried_while_disconnected_then_cleared() {
        let (mut dap, probe) = probe_offering(VERGE_VENDOR, &[1, 2]);
        assert_eq!(
            dap.cjtag_scan_formats().unwrap(),
            [ScanFormat::OScan1, ScanFormat::OScan2]
        );
        let sent = sent(&probe);
        assert_eq!(
            sent.last().unwrap(),
            &[0x7F, 6, 0x03, 0x95, 1, 0x95, 2, 0x95, 5, 0x95, 6, 0x95, 0]
        );
        assert!(!sent.iter().any(|command| command.first() == Some(&0x02)));
    }

    #[test]
    fn a_reset_walk_after_the_activation_sends_nothing() {
        let (mut dap, probe) = attached();
        let before = sent(&probe).len();
        dap.send_jtag_sequences(sequences(true, 6)).unwrap();
        assert_eq!(sent(&probe).len(), before);
    }

    #[test]
    fn a_walk_into_test_logic_reset_activates_the_target_again() {
        let (mut dap, probe) = attached();
        let before = sent(&probe).len();

        // Out of Test-Logic-Reset into Run-Test/Idle, where the activation left the TAP.
        dap.send_jtag_sequences(sequences(false, 1)).unwrap();
        assert_eq!(sent(&probe).len(), before);

        // Test-Logic-Reset at the third clock; the two after it would hold the TAP there.
        dap.send_jtag_sequences(sequences(true, 5)).unwrap();
        let sent = sent(&probe);
        assert_eq!(sent[before], [0x14, 1, 0x40 | 3, 0]);
        assert_eq!(sent[before + 1], [0x96, 10, 1]);
        // The activation's last command, leaving the command level; nothing follows it.
        assert_eq!(
            sent.last().unwrap(),
            &[0x14, 4, 0x42, 0, 0x01, 0, 0x42, 0, 0x01, 0]
        );
    }

    #[test]
    fn a_transfer_leaves_the_tap_where_the_activation_did() {
        let (mut dap, probe) = attached();
        // The walk to Run-Test/Idle before a transfer ended where the activation left the TAP.
        dap.jtag_state.tap_state = TapState::RunTestIdle;
        let before = sent(&probe).len();

        // So the TMS-high clocks are no longer dropped: the third reaches Test-Logic-Reset.
        dap.send_jtag_sequences(sequences(true, 3)).unwrap();
        let sent = sent(&probe);
        assert_eq!(sent[before], [0x14, 1, 0x40 | 3, 0]);
        assert_eq!(sent[before + 1], [0x96, 10, 1]);
    }

    #[test]
    fn a_swj_sequence_goes_to_the_target_framed() {
        let (mut dap, probe) = attached();
        let before = sent(&probe).len();

        let mut batch = crate::probe::swd::SwdBatch::new();
        batch.sequence(crate::probe::BitSequence::from_u64(16, 0xE3BC));
        crate::probe::swd::SwdProbe::run_batch(&mut dap, &batch).unwrap();

        let sent = sent(&probe);
        assert_eq!(sent[before][0], 0x14);
        assert!(!sent.contains(&vec![0x12, 16, 0xBC, 0xE3]));
    }

    #[test]
    fn a_disconnect_in_oscan2_completes_the_last_move_first() {
        for (format, frames) in [(ScanFormat::OScan1, 0), (ScanFormat::OScan2, 1)] {
            let (mut dap, probe) = self::probe(VERGE_VENDOR);
            dap.set_cjtag_scan_format(format);
            dap.select_protocol(WireProtocol::CJtag).unwrap();
            dap.attach().unwrap();
            let before = sent(&probe).len();
            dap.detach().unwrap();
            let sent = sent(&probe);
            let disconnect = sent.iter().rposition(|command| command == &[0x03]).unwrap();
            let idle = [0x14, 1, 0x01, 0];
            let count = sent[before..disconnect]
                .iter()
                .filter(|c| **c == idle)
                .count();
            assert_eq!(count, frames, "{format:?}");
        }
    }

    #[test]
    fn only_verge_is_asked_about_cjtag() {
        let (mut dap, probe) = self::probe("ARM");
        assert!(dap.select_protocol(WireProtocol::CJtag).is_err());
        assert!(!sent(&probe).iter().any(|command| command[0] == 0x82));
    }

    /// The Shift-DR clocks of each DR scan in `tms`, from Run-Test/Idle.
    fn dr_scans(tms: &BitSlice) -> Vec<usize> {
        let mut state = FullTapState::RunTestIdle;
        let mut scans = Vec::new();
        let mut shifts = 0;
        for bit in tms.iter().by_vals() {
            if state == FullTapState::ShiftDr {
                shifts += 1;
            }
            state = state.step(bit);
            if state == FullTapState::UpdateDr {
                scans.push(std::mem::take(&mut shifts));
            }
        }
        assert_eq!(state, FullTapState::RunTestIdle);
        scans
    }

    #[test]
    fn the_oscan1_entry_is_the_reference_long_form() {
        let expected = tms(&[
            "1011010110100110100110100110100001101",
            &"0".repeat(10),
            "1100000",
        ]);
        assert_eq!(enter_format(ScanFormat::OScan1), expected);
    }

    #[test]
    fn each_entry_locks_the_command_level_then_stores_its_format() {
        // SCNFMT values, IEEE 1149.7 Table 23-2.
        let formats = [
            (ScanFormat::OScan1, 9),
            (ScanFormat::OScan2, 10),
            (ScanFormat::OScan5, 13),
            (ScanFormat::OScan6, 14),
        ];
        for (format, scnfmt) in formats {
            // Two zero-bit scans, the one-clock lock, STC1 1, STFMT and its SCNFMT value.
            assert_eq!(
                dr_scans(&enter_format(format)),
                [0, 0, 1, 1, 1, 3, scnfmt],
                "{format:?}"
            );
        }
    }

    #[test]
    fn a_long_alternating_walk_is_split_to_fit_the_packet() {
        let tms: BitVec = (0..128).map(|i| i % 2 == 0).collect();
        let requests = tms_requests(&tms, true, 64).unwrap();
        assert_eq!(
            requests.iter().map(|r| r.sequences().len()).sum::<usize>(),
            128
        );
        assert!(requests.iter().all(|r| 2 + 2 * r.sequences().len() <= 64));
    }
}
