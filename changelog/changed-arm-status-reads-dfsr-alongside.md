Reading a Cortex-M core's status now fetches the fault status register alongside the halt status
rather than after it. The extra read rides along in the same request, so a halt costs one round
trip to the probe where it previously cost two.
