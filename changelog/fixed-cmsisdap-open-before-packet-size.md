Fixed opening a CMSIS-DAP v1 probe hanging when the probe uses report sizes larger than the default
guess. Recovery traffic for a desynchronised probe was sent before the packet size had been
discovered, so probes that answer nothing until they receive a full report answered nothing at all.
