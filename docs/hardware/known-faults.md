# Known faults
List of known hardware faults in V0.0 hardware.


## Faults
:::{warning}
i2c line is missing pull-ups
:::

:::{warning}
USB-C power pulldowns are actually pullups
:::

:::{warning}
Silkscreen has incorrect polarity on XT30 connector
:::

:::{warning}
The 5V->3.3V regulator and DRV 3.3V regulator are hooked into the same net with no diode. This doesn't directly seem to cause any problems, other than the drive not turning off when you remove 5V power but keep motor power on. Future iterations will likely just not attach the DRV regulator output, as there is minimal need for the MCU to operate when it does not have either a CAN or USB connection.
:::

## Unknown source faults

:::{warning}
Seeing what I believe is a thermal shutdown on the DRV chip at higher voltages and speeds. Guessing it's the drive level set too high. Need to diagnose.
:::