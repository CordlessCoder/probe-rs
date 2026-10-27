CMSIS-DAP v1 probes take their report size from the HID report descriptor, so on Linux a probe with reports bigger than 64 bytes opens on the first try.
