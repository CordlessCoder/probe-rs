//! Sequences for TI MSPM0 devices.
//!
//! The MSPM0's AHB-AP lives in power domain PD1. Entering DEEPSLEEP (STOP or STANDBY) disables
//! PD1, which makes the AHB-AP undiscoverable and drops the debug session. This is not limited to
//! applications that sleep deliberately: a blank device parks itself in STANDBY0 after roughly ten
//! seconds of bootcode, which revokes the AHB-AP the same way.
//!
//! The override is the PWR-AP (APSEL 4). Setting `INHIBITSLEEP` and `FORCEACTIVE` in its `DPREC0`
//! register forces the device out of low-power mode and keeps it out for as long as the debugger
//! is attached. TI documents this as mandatory when adding MSPM0 support ("Hardware Programming
//! and Debugger Guide for MSPM0", SLAAEO5 section 3.1).
//!
//! The values here are transcribed from TI's own low-power-mode patches shipped in the MSPM0 SDK
//! (`tools/keil/low_power_mode_patch/*.pdsc` `DebugPortStart` sequences, cross-checked against
//! `tools/iar/low_power_mode_patch/*.dmac` `_InhibitSleepForceActive()`).
//!
//! Resets go through SYSCTL when the caller has armed the reset catch, because `AIRCR.SYSRESETREQ`
//! only resets the CPU here and leaves the peripherals as the flash algorithm left them. Without
//! the catch the stock reset is used instead — see [`MSPM0::reset_system`].

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use crate::architecture::arm::ap::{ApRegister, IDR};
use crate::architecture::arm::core::armv7m::Demcr;
use crate::architecture::arm::dp::DpAddress;
use crate::architecture::arm::memory::ArmMemoryInterface;
use crate::architecture::arm::sequences::{
    ArmDebugSequence, ArmDebugSequenceError, cortex_m_reset_system, cortex_m_wait_for_reset,
};
use crate::architecture::arm::traits::Pins;
use crate::architecture::arm::{ArmDebugInterface, ArmError, DapAccess, FullyQualifiedApAddress};
use crate::core::MemoryMappedRegister;
use crate::session::MissingPermissions;
use probe_rs_target::CoreType;

/// Access Port Select values used by this sequence.
#[derive(Debug, Clone, Copy)]
enum ApSel {
    /// AHB-AP: memory access to the core. It lives in power domain PD1, so it is missing while
    /// the device is in a low-power state.
    Ahb = 0,
    /// SEC-AP: the mailbox the boot ROM services, used to recover an inaccessible device.
    Sec = 2,
    /// PWR-AP: controls the power and reset state of the CPU for debug purposes.
    Pwr = 4,
}

impl From<ApSel> for FullyQualifiedApAddress {
    fn from(apsel: ApSel) -> Self {
        FullyQualifiedApAddress::v1_with_default_dp(apsel as u8)
    }
}

/// Debug power and reset control register, PWR-AP register bank 0.
const DPREC0: u64 = 0x00;
/// System power and reset control register, PWR-AP register bank 15.
///
/// The AP bank is derived from the register address by the communication interface, so this can be
/// addressed as a plain offset.
const SPREC: u64 = 0xF0;

/// `DPREC0.FORCEACTIVE` — force the device out of a low-power state.
const DPREC0_FORCEACTIVE: u32 = 1 << 3;
/// `DPREC0.RST CTL` (bits 16:14) set to `100b`, selecting halt-on-reset.
const DPREC0_HALT_ON_RESET: u32 = 0b100 << 14;
/// The whole `DPREC0.RST CTL` field. Its default is `000b` (SLAAEO5 table 3-2).
const DPREC0_RST_CTL: u32 = 0b111 << 14;
/// `DPREC0.DEBUGPOWER`.
///
/// Documented as Reserved in SLAAEO5 table 3-3, but set by both of TI's toolchain patches.
const DPREC0_DEBUGPOWER: u32 = 1 << 19;
/// `DPREC0.INHIBITSLEEP` — refuse requests to enter DEEPSLEEP.
const DPREC0_INHIBITSLEEP: u32 = 1 << 20;

/// Bits 23:21 of `DPREC0`.
///
/// Undocumented. TI's patches call these the "sticky" bits and take a recovery path when any of
/// them is set, so we do the same.
const DPREC0_STICKY: u32 = 0x00E0_0000;

/// The steady-state value TI's patches write to `DPREC0`.
const DPREC0_DEBUG_ENABLE: u32 =
    DPREC0_FORCEACTIVE | DPREC0_HALT_ON_RESET | DPREC0_DEBUGPOWER | DPREC0_INHIBITSLEEP;

/// `SPREC.SYS RST`.
const SPREC_SYS_RST: u32 = 1 << 0;

/// SEC-AP `TXDATA`, the word passed alongside a mailbox command.
const TXDATA: u64 = 0x00;
/// SEC-AP `TXCTL`. The command goes here as-is; bit 0 is `TXVLD`, which the ROM drives.
const TXCTL: u64 = 0x04;
/// SEC-AP `RXDATA`, the boot ROM's answer.
const RXDATA: u64 = 0x08;
/// SEC-AP `RXCTL`. Bit 0 is `RXVLD`, and reading `RXDATA` clears it.
const RXCTL: u64 = 0x0C;

/// `RXCTL.RXVLD`.
const RXCTL_RX_VALID: u32 = 1 << 0;

/// DSSM "Mass Erase": erases MAIN and leaves NONMAIN, and so the debug access policy, alone.
///
/// The alternative, "Factory Reset" (`0x020A`), also erases NONMAIN and restores TI's defaults.
/// That is a bigger hammer than an unreachable AHB-AP warrants.
const DSSM_MASS_ERASE: u32 = 0x020C;
/// DSSM "Factory Reset": erases MAIN *and* NONMAIN and repopulates NONMAIN with TI's defaults.
///
/// The only documented way back from a NONMAIN the boot code cannot validate — a bad `userCfgCRC`
/// leaves the ROM refusing to start the application, enable debug, or invoke the BSL, and it
/// honours a pending factory reset because it pattern-matches that field rather than trusting the
/// structure it could not check (SLAU847 1.4.1.1).
const DSSM_FACTORY_RESET: u32 = 0x020A;
/// What the boot ROM leaves in `RXDATA` when it has serviced a command.
const DSSM_RESPONSE_OK: u32 = 0x0001_0003;

/// `SYSCTL.RESETLEVEL` — selects the level of the next software-triggered reset.
const SYSCTL_RESETLEVEL: u64 = 0x400B_0300;
/// `SYSCTL.RESETCMD` — executes the reset selected by `RESETLEVEL`.
const SYSCTL_RESETCMD: u64 = 0x400B_0304;

/// `RESETLEVEL.LEVEL` = 0: SYSRST, resetting the CPU and the peripherals.
///
/// Level 1 (BOOTRST) would additionally run the boot configuration routine, which parks a blank
/// device in STANDBY0.
const RESETLEVEL_SYSRST: u32 = 0;
/// `RESETCMD.GO` together with the `RESETCMD.KEY` value of `0xE4` it has to be written with.
const RESETCMD_GO: u32 = 0xE400_0001;

/// Marker struct indicating initialization sequencing for MSPM0 family parts.
#[derive(Debug)]
pub struct MSPM0 {
    /// Chip name, used to select the recovery variant.
    name: String,
    /// Whether this part needs the longer sticky-bit recovery sequence.
    long_recovery: bool,
}

impl MSPM0 {
    /// Create the sequencer for the MSPM0 family of parts.
    pub fn create(name: String) -> Arc<Self> {
        // TI ships two flavours of the recovery path. The MSPM0C110X and MSPS003FX packs use a
        // longer variant; every other family uses the short one.
        let long_recovery = name.starts_with("MSPM0C110") || name.starts_with("MSPS003F");

        Arc::new(Self {
            name,
            long_recovery,
        })
    }

    /// Read `DPREC0` and log it, mirroring the `Message()` calls in TI's debug sequences.
    fn read_dprec0(&self, interface: &mut dyn DapAccess) -> Result<u32, ArmError> {
        let pwr_ap: FullyQualifiedApAddress = ApSel::Pwr.into();
        let value = interface.read_raw_ap_register(&pwr_ap, DPREC0)?;
        tracing::debug!("{}: DPREC0 is {:#010x}", self.name, value);
        Ok(value)
    }

    fn write_dprec0(&self, interface: &mut dyn DapAccess, value: u32) -> Result<(), ArmError> {
        let pwr_ap: FullyQualifiedApAddress = ApSel::Pwr.into();
        interface.write_raw_ap_register(&pwr_ap, DPREC0, value)
    }

    fn write_sprec(&self, interface: &mut dyn DapAccess, value: u32) -> Result<(), ArmError> {
        let pwr_ap: FullyQualifiedApAddress = ApSel::Pwr.into();
        interface.write_raw_ap_register(&pwr_ap, SPREC, value)
    }

    /// Whether the AHB-AP answers.
    ///
    /// `FORCEACTIVE` needs a moment to bring PD1 back up, so this polls rather than reading once.
    /// Treating a slow wake as an absent AP would put us straight back into resetting a device
    /// that was only asleep.
    fn ahb_ap_responds(&self, interface: &mut dyn DapAccess) -> bool {
        let ahb_ap: FullyQualifiedApAddress = ApSel::Ahb.into();

        let start = Instant::now();
        loop {
            if matches!(interface.read_raw_ap_register(&ahb_ap, IDR::ADDRESS), Ok(idr) if idr != 0)
            {
                return true;
            }
            if start.elapsed() > Duration::from_millis(10) {
                return false;
            }
            thread::sleep(Duration::from_millis(1));
        }
    }

    /// Recover a device whose `DPREC0` sticky bits are set.
    ///
    /// The meaning of bits 23:21 is undocumented; this reproduces what TI's packs do.
    fn recover_sticky(&self, interface: &mut dyn DapAccess) -> Result<(), ArmError> {
        tracing::warn!(
            "{}: DPREC0 sticky bits are set, running the PWR-AP recovery sequence",
            self.name
        );

        self.write_sprec(interface, SPREC_SYS_RST)?;

        if self.long_recovery {
            self.read_dprec0(interface)?;
            self.write_dprec0(interface, DPREC0_FORCEACTIVE)?;

            self.read_dprec0(interface)?;
            self.write_dprec0(interface, DPREC0_DEBUG_ENABLE | DPREC0_STICKY)?;

            self.read_dprec0(interface)?;
            self.write_sprec(interface, SPREC_SYS_RST)?;
            self.write_dprec0(interface, DPREC0_DEBUG_ENABLE)?;
        } else {
            // Writing the sticky bits back preserves them, as TI's packs do.
            self.write_dprec0(interface, DPREC0_DEBUG_ENABLE | DPREC0_STICKY)?;
        }

        self.read_dprec0(interface)?;

        Ok(())
    }

    /// Run one boot ROM mailbox command, named by `what` for the error messages.
    ///
    /// The mailbox is only read by the boot code, and only out of a BOOTRST, so the command has to
    /// be staged before the reset rather than issued after it. A system reset through the PWR-AP is
    /// a lower reset level and does not run the boot code, which leaves the reset pin as the only
    /// way in when the AHB-AP is gone (SLAAEO5 section 4).
    fn dssm_command(
        &self,
        interface: &mut dyn ArmDebugInterface,
        command: u32,
        what: &str,
    ) -> Result<(), ArmError> {
        let sec_ap: FullyQualifiedApAddress = ApSel::Sec.into();

        interface.write_raw_ap_register(&sec_ap, TXCTL, command)?;
        interface.write_raw_ap_register(&sec_ap, TXDATA, 0)?;

        // Drain anything an earlier command left in the receive side, or its answer will be
        // mistaken for this one's.
        let _ = interface.read_raw_ap_register(&sec_ap, RXDATA);
        let _ = interface.read_raw_ap_register(&sec_ap, RXCTL);
        let _ = interface.flush();
        thread::sleep(Duration::from_millis(500));

        let mut pins = Pins(0);
        pins.set_nreset(true);
        interface.swj_pins(0, pins.0 as u32, 0)?;
        thread::sleep(Duration::from_millis(50));
        interface.swj_pins(pins.0 as u32, pins.0 as u32, 0)?;

        let start = Instant::now();
        loop {
            let rxctl = interface.read_raw_ap_register(&sec_ap, RXCTL).unwrap_or(0);
            if rxctl & RXCTL_RX_VALID != 0 {
                break;
            }
            if start.elapsed() > Duration::from_secs(2) {
                return Err(ArmDebugSequenceError::custom(format!(
                    "MSPM0: the boot ROM did not answer the {what} command"
                ))
                .into());
            }
            thread::sleep(Duration::from_millis(1));
        }

        // Reading RXDATA clears RXVLD, so this order matters.
        let response = interface.read_raw_ap_register(&sec_ap, RXDATA)?;
        let echoed = interface.read_raw_ap_register(&sec_ap, RXCTL)?;

        if response != DSSM_RESPONSE_OK || echoed != command & 0xFF {
            return Err(ArmDebugSequenceError::custom(format!(
                "MSPM0: {what} rejected, RXDATA {response:#010x} RXCTL {echoed:#010x}"
            ))
            .into());
        }

        Ok(())
    }

    /// Reset NONMAIN to TI's defaults, recovering a device the boot code will not start.
    fn dssm_factory_reset(&self, interface: &mut dyn ArmDebugInterface) -> Result<(), ArmError> {
        self.dssm_command(interface, DSSM_FACTORY_RESET, "factory reset")?;

        // Warn rather than inform: the default stderr filter is `WARN`, so an `info!` reporting
        // success is invisible beside any warning that preceded it, and a recovery then reads as a
        // failure. The caveat is the same one mass erase carries — nothing is running yet.
        tracing::warn!(
            "{}: NONMAIN reset to factory defaults and main flash erased. Both are blank now, so \
             the core will fault as soon as it is released — program the device in this session, \
             or it will be unreachable again.",
            self.name
        );

        Ok(())
    }
}

impl ArmDebugSequence for MSPM0 {
    fn debug_port_start(
        &self,
        interface: &mut dyn DapAccess,
        dp: DpAddress,
    ) -> Result<(), ArmError> {
        self.debug_port_start_default(interface, dp)?;

        // Everything below is specific to MSPM0: keep the device out of DEEPSLEEP for as long as
        // we are attached, otherwise the AHB-AP disappears along with power domain PD1.
        //
        // A failure here is not recoverable by us: if `DEBUGSS.SPECIAL_AUTH.PWRAPEN` is deasserted
        // in NONMAIN, a DAPBUS firewall isolates the PWR-AP entirely. Let the error propagate
        // rather than continuing into a session that will drop a few seconds later.
        let dprec0 = self.read_dprec0(interface)?;

        if dprec0 & DPREC0_STICKY == 0 {
            self.write_dprec0(interface, DPREC0_DEBUG_ENABLE)?;
            return Ok(());
        }

        // The sticky bits are set on any device that has been in a low-power state, not only on
        // one that needs recovering, so finding them set is not on its own a reason to reset the
        // device. Write what the recovery would write and see whether the AHB-AP comes back:
        // `FORCEACTIVE` is what makes it discoverable again (SLAAEO5 section 3.1). Only when it
        // stays missing is there something wrong, and only then is a system reset worth its cost.
        self.write_dprec0(interface, DPREC0_DEBUG_ENABLE | DPREC0_STICKY)?;

        if self.ahb_ap_responds(interface) {
            return Ok(());
        }

        self.recover_sticky(interface)
    }

    fn debug_core_stop(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        core_type: CoreType,
    ) -> Result<(), ArmError> {
        // Let the core be torn down normally first; the PWR-AP write below is what lets the device
        // sleep again, so it has to come last.
        self.debug_core_stop_default(interface, core_type)?;

        let interface = interface.get_arm_debug_interface()?;

        // Hand low-power control back to the application. Leaving INHIBITSLEEP set would keep the
        // part awake and burning current until its next reset.
        //
        // Put RST CTL back to its default too. Halt-on-reset applies to "any form of reset
        // performed on it post-configuration" (SLAAEO5 3.2.2), so leaving it selected stops the
        // core on the next reset with no debugger present to release it. `debug_port_start`
        // selects it again on the next attach.
        let dprec0 = self.read_dprec0(interface)?;
        self.write_dprec0(
            interface,
            dprec0 & !(DPREC0_INHIBITSLEEP | DPREC0_FORCEACTIVE | DPREC0_RST_CTL),
        )?;

        Ok(())
    }

    fn debug_device_unlock(
        &self,
        interface: &mut dyn ArmDebugInterface,
        default_ap: &FullyQualifiedApAddress,
        permissions: &crate::Permissions,
    ) -> Result<(), ArmError> {
        // Everything past this point needs the AHB-AP. If it answers, there is nothing to recover.
        if matches!(interface.read_raw_ap_register(default_ap, IDR::ADDRESS), Ok(idr) if idr != 0) {
            return Ok(());
        }

        tracing::warn!(
            "{}: the AHB-AP is not responding. Either the device is in a low-power mode that \
             disables PD1, or main flash holds an image that faults the core as soon as it runs.",
            self.name
        );

        // The only way back is to erase what the core is running, so this needs saying out loud
        // rather than happening quietly on an ordinary attach.
        permissions
            .erase_all()
            .map_err(|MissingPermissions(desc)| ArmError::MissingPermissions(desc))?;

        self.dssm_command(interface, DSSM_MASS_ERASE, "mass erase")?;

        // Warn rather than inform, for two reasons. The default stderr filter is `WARN`, so an
        // `info!` here is invisible next to the warnings above it and the recovery reads as a
        // failure. And the device is not out of trouble yet: main flash is blank, which is itself
        // an image the core faults on, so a session that ends here leaves the part exactly as
        // unreachable as it found it.
        tracing::warn!(
            "{}: main flash erased, device recovered. It is blank now, so the core will fault as \
             soon as it is released — program the device in this session, or it will be \
             unreachable again.",
            self.name
        );

        Err(ArmError::ReAttachRequired)
    }

    fn factory_reset(&self, interface: &mut dyn ArmDebugInterface) -> Result<(), ArmError> {
        self.dssm_factory_reset(interface)?;

        // The device has restarted out of a BOOTRST and nothing has been programmed, so whatever
        // the caller had is gone along with the connection it had it through.
        Err(ArmError::ReAttachRequired)
    }

    fn reset_system(
        &self,
        interface: &mut dyn ArmMemoryInterface,
        _core_type: CoreType,
        _debug_base: Option<u64>,
    ) -> Result<(), ArmError> {
        // `CPUSS.CTL` holds the prefetch and cache enables and resets to 0x7 (SLAU847F 3.6.15).
        // TI's flash algorithms clear it in `Init` and do not restore it, so once programming
        // has finished the prefetcher and both caches are off and every flash access is slower.
        // The TRM does not say which reset level restores the register. `AIRCR.SYSRESETREQ`
        // leaves it as the algorithm left it; SYSCTL's SYSRST puts it back (SLAAEO5 section 6).
        //
        // Only do that when the caller has armed the reset catch. SYSRST restarts the core, which
        // then runs from whatever the reset vector holds; on a device with erased MAIN that is
        // 0xFFFFFFFF, and the core faults, locks up and takes the access ports down with it.
        // `reset_and_halt` arms the catch and is the path flashing resets through, so SYSRST is
        // both needed and safe there. A bare `Core::reset` is neither, and gets the stock reset
        // that probe-rs used before.
        let demcr = Demcr(interface.read_word_32(Demcr::get_mmio_address())?);
        if !demcr.vc_corereset() {
            return cortex_m_reset_system(interface);
        }

        interface.write_word_32(SYSCTL_RESETLEVEL, RESETLEVEL_SYSRST)?;

        // The device resets while this write is still in flight, so its acknowledge is unreliable.
        interface.write_word_32(SYSCTL_RESETCMD, RESETCMD_GO).ok();
        interface.flush().ok();

        cortex_m_wait_for_reset(interface)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The values TI's Keil packs and IAR macros write, pinned so a change to the bit definitions
    /// above cannot silently alter what goes on the wire.
    #[test]
    fn dprec0_values_match_ti_patches() {
        assert_eq!(DPREC0_DEBUG_ENABLE, 0x0019_0008);
        assert_eq!(DPREC0_DEBUG_ENABLE | DPREC0_STICKY, 0x00F9_0008);
        assert_eq!(DPREC0_FORCEACTIVE, 0x0000_0008);
    }

    /// `debug_core_stop` has to clear the whole RST CTL field, not just the bit `DPREC0_DEBUG_ENABLE`
    /// happens to set, or the device is left in some other reset mode instead of the default.
    #[test]
    fn clearing_rst_ctl_selects_the_default_reset_mode() {
        assert_eq!(DPREC0_HALT_ON_RESET & DPREC0_RST_CTL, DPREC0_HALT_ON_RESET);
        assert_eq!(DPREC0_DEBUG_ENABLE & !DPREC0_RST_CTL, 0x0018_0008);
    }

    /// The SYSCTL reset command from SLAAEO5 table 6-1, pinned so the key cannot go missing.
    #[test]
    fn sysctl_reset_matches_ti_documentation() {
        assert_eq!(SYSCTL_RESETLEVEL, 0x400B_0300);
        assert_eq!(SYSCTL_RESETCMD, 0x400B_0304);
        assert_eq!(RESETLEVEL_SYSRST, 0);
        assert_eq!(RESETCMD_GO, 0xE400_0001);
    }

    #[test]
    fn long_recovery_is_limited_to_c110x_and_msps003fx() {
        for name in ["MSPM0C1103", "MSPM0C1104", "MSPS003F3", "MSPS003F4"] {
            assert!(
                MSPM0::create(name.to_string()).long_recovery,
                "{name} should use the long recovery sequence"
            );
        }

        for name in ["MSPM0L1306", "MSPM0L2228", "MSPM0G3507", "MSPM0G3519"] {
            assert!(
                !MSPM0::create(name.to_string()).long_recovery,
                "{name} should use the short recovery sequence"
            );
        }
    }

    /// Guards the chip-name prefix matching in `vendor/ti/mod.rs`: every built-in MSPM0 target must
    /// resolve to this sequence rather than falling through to the generic ARM default.
    #[test]
    fn all_builtin_mspm0_targets_get_the_mspm0_sequence() {
        let registry = crate::config::Registry::from_builtin_families();

        let names = [
            "MSPM0C1104",
            "MSPM0L1306",
            "MSPM0L2117",
            "MSPM0L2228",
            "MSPM0G3507",
            "MSPM0G5187",
            "MSPM0G3519",
        ];

        for name in names {
            let target = registry
                .get_target_by_name(name)
                .unwrap_or_else(|e| panic!("{name} is not a built-in target: {e}"));

            let debug_sequence = format!("{:?}", target.debug_sequence);
            assert!(
                debug_sequence.contains("MSPM0"),
                "{name} resolved to {debug_sequence}, expected the MSPM0 sequence"
            );
        }
    }
}
