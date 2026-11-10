//! Sequences for cc13xx_cc26xx devices
use std::sync::Arc;
use std::time::Duration;

use crate::MemoryMappedRegister;
use crate::architecture::arm::armv7m::{Demcr, Dhcsr};
use crate::architecture::arm::communication_interface::probe_debug_port_wire;
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{ArmDebugSequence, ArmDebugSequenceError};
use crate::architecture::arm::traits::DebugPortWire;
use crate::architecture::arm::{
    ArmDebugInterface, ArmError, FullyQualifiedApAddress, dp::DpAddress,
};
use crate::probe::{BitSequence, DebugProbeError, WireProtocol};
use probe_rs_target::CoreType;

use super::icepick::{DefaultProtocol, Icepick};

/// Marker struct indicating initialization sequencing for cc13xx_cc26xx family parts.
#[derive(Debug)]
pub struct CC13xxCC26xx {
    // Chip name
    name: String,
}

impl CC13xxCC26xx {
    /// Create the sequencer for the cc13xx_cc26xx family of parts.
    pub fn create(name: String) -> Arc<Self> {
        Arc::new(Self { name })
    }
}

/// Do a full system reset (emulated PIN reset)
///
/// CPU reset alone is not possible since AIRCR.SYSRESETREQ will be
/// converted to system reset on these devices.
///
/// The below code writes to the following bit
/// `AON_PMCTL.RESETCTL.SYSRESET=1`d or its equivalent based on family
fn reset_chip(chip: &str, probe: &mut dyn ArmMemoryInterface) {
    // The CC family of device have a pattern where the 6th character of the device name dictates the family
    // Use this to determine the correct address to write to
    match chip.chars().nth(5).unwrap() {
        // Note that errors are ignored
        // writing this register will immediately trigger a system reset which causes us to lose the debug interface
        // We also don't need to worry about preserving register state because we will anyway reset.
        '0' => {
            probe.write_word_32(0x4009_0004, 0x8000_0000).ok();
        }
        '1' | '2' => {
            probe.write_word_32(0x4009_0028, 0x8000_0000).ok();
        }
        '4' => {
            probe.write_word_32(0x5809_0028, 0x8000_0000).ok();
        }
        _ => {
            unreachable!(
                "TI CC13xx/CC26xx debug sequence used on an unsupported chip: {chip}",
                chip = chip
            );
        }
    }
}

impl CC13xxCC26xx {
    /// The core type, from the family digit of the part name, as [`reset_chip`] reads it.
    fn core_type(&self) -> CoreType {
        match self.name.chars().nth(5) {
            Some('0') => CoreType::Armv7m,
            Some('4') => CoreType::Armv8m,
            _ => CoreType::Armv7em,
        }
    }

    /// Whether test TAP 5 is the WUC TAP that [`Icepick::reset_through_wuc`] writes. On CC13x4 and
    /// CC26x4 it is AON_PMCTL, whose registers are not documented (SWCU194 Table 6-4).
    fn has_wuc_tap(&self) -> bool {
        matches!(self.name.chars().nth(5), Some('0' | '1' | '2'))
    }
}

fn halt(probe: &mut dyn ArmMemoryInterface) -> Result<(), ArmError> {
    // TODO! Find a way to call the armv7m::halt function instead
    let mut value = Dhcsr(0);
    value.set_c_halt(true);
    value.set_c_debugen(true);
    value.enable_write();

    probe.write_word_32(Dhcsr::get_mmio_address(), value.into())
}

impl ArmDebugSequence for CC13xxCC26xx {
    /// Asserts nothing. A pin reset turns the JTAG power domain off (SWCU185G 6.4), so nothing
    /// can attach while the pin is held. [`Self::reset_hardware_deassert`] resets the chip instead.
    fn reset_hardware_assert(&self, _interface: &mut dyn DebugPortWire) -> Result<(), ArmError> {
        Ok(())
    }

    /// Resets the chip and halts the core before any application code runs.
    ///
    /// The attach powered the JTAG domain up, which sets Halt In Boot: after any reset other than
    /// a pin reset or power-on, the boot ROM waits for the debugger before it starts the
    /// application (SWCU185G 6.4, 6.6). The reset goes through the CPU's access port, or, on parts
    /// with a WUC TAP, through that TAP when the port does not answer.
    fn reset_hardware_deassert(
        &self,
        interface: &mut dyn ArmDebugInterface,
        default_ap: &FullyQualifiedApAddress,
    ) -> Result<(), ArmError> {
        let core_type = self.core_type();
        let reset = interface
            .memory_interface(default_ap)
            .and_then(|mut memory| self.reset_system(&mut *memory, core_type, None));
        let Err(error) = reset else {
            return Ok(());
        };
        if !self.has_wuc_tap() {
            return Err(error);
        }
        tracing::warn!("Unable to reset through the CPU's access port, using the WUC TAP: {error}");

        let driver = interface.driver_mut().ok_or(ArmError::Probe(
            DebugProbeError::InterfaceNotAvailable {
                interface_name: "JTAG",
            },
        ))?;
        probe_debug_port_wire(driver, |wire| {
            Icepick::new(wire, DefaultProtocol::CJtag)?.reset_through_wuc()
        })?;

        std::thread::sleep(Duration::from_millis(1));
        interface.reinitialize()?;
        self.debug_core_start(interface, default_ap, core_type, None, None)?;
        halt(&mut *interface.memory_interface(default_ap)?)
    }

    fn reset_system(
        &self,
        probe: &mut dyn ArmMemoryInterface,
        core_type: probe_rs_target::CoreType,
        debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        // Check if the previous code requested a halt before reset
        let demcr = Demcr(probe.read_word_32(Demcr::get_mmio_address())?);

        // Do target specific reset
        reset_chip(&self.name, probe);

        // Since the system went down, including the debug, we should flush any pending operations
        probe.flush().ok();

        // Wait for the system to reset
        std::thread::sleep(Duration::from_millis(1));

        // Re-initializing the core(s) is on us.
        let ap = probe.fully_qualified_address();
        let interface = probe.get_arm_debug_interface()?;
        interface.reinitialize()?;

        assert!(debug_base.is_none());
        self.debug_core_start(interface, &ap, core_type, None, None)?;

        if demcr.vc_corereset() {
            halt(probe)?;
        }

        Ok(())
    }

    fn debug_port_setup(
        &self,
        interface: &mut dyn DebugPortWire,
        _dp: DpAddress,
    ) -> Result<(), ArmError> {
        // Ensure current debug interface is in reset state.
        interface.swj_sequence(&BitSequence::from_u64(51, 0x0007_FFFF_FFFF_FFFF))?;

        match interface.active_protocol() {
            Some(WireProtocol::Jtag | WireProtocol::CJtag) => {
                let mut icepick = Icepick::new(interface, DefaultProtocol::CJtag)?;
                icepick.select_tap(0, &self.name)?;

                // Call the configure JTAG function. We don't derive the scan chain at runtime
                // for these devices, but regardless the scan chain must be told to the debug probe
                // We avoid the live scan for the following reasons:
                // 1. Only the ICEPICK is connected at boot so we need to manually the CPU to the scan chain
                // 2. Entering test logic reset disconnects the CPU again
                interface.configure_jtag(true)?;
            }
            Some(WireProtocol::Swd) => {
                return Err(ArmDebugSequenceError::SequenceSpecific(
                    "The cc13xx_cc26xx family doesn't support SWD".into(),
                )
                .into());
            }
            _ => {
                return Err(ArmDebugSequenceError::SequenceSpecific(
                    "Cannot detect current protocol".into(),
                )
                .into());
            }
        }

        Ok(())
    }
}
