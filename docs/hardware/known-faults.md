# Known faults
List of known hardware faults in V0.0 hardware.


## Root cause faults
:::{warning}
i2c line is missing pull-ups
:::

:::{warning}
USB-C power pulldowns are actually pullups
:::

:::{warning}
Silkscreen has incorrect polarity on XT30 connector
:::

## Unknown source faults

:::{warning}
Seeing what I believe is a thermal shutdown on the DRV chip at higher voltages and speeds. Guessing it's the drive level set too high. Need to diagnose.
:::