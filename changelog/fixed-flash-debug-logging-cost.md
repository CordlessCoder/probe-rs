Enabling debug logging no longer slows flashing down. The flash algorithm's registers were read back
after being written whenever anything listened at that level, an extra probe round trip per register
on every sector erased and page programmed; the readback is now at trace level.
