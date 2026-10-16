CMSIS-DAP probes keep transfer requests queued across a whole batch, so a memory access longer than 1 KiB no longer waits for the probe at every address window.
