Staging a flash algorithm's core registers now sends the whole set to the probe in one request.
Written one at a time, each register cost a round trip for the ready flag that the next one waited
on. Added `CoreInterface::write_core_regs` for it.
