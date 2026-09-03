Enabling debug logging no longer slows flashing down. The flash algorithm's registers were read back
after being written whenever anything listened at that level, four extra probe round trips per
sector erased and page programmed; the readback is now at trace level.
