MSPM0: attaching no longer resets a device that has been in a low-power state. The `DPREC0` sticky
bits are set by ordinary low-power entry, and the recovery they triggered begins with a system
reset. The recovery now runs only when the AHB-AP stays missing after `FORCEACTIVE` is asserted.
