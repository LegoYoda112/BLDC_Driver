# Drive
Drive.c includes all motor drive related functions, such as control loops and FOC. This might get split into separate axis control and FOC files.

## Variables
```{eval-rst}
.. doxygenvariable:: enc_angle_int
.. doxygenvariable:: target_encoder_value
.. doxygenvariable:: electrical_angle
.. doxygenvariable:: electrical_angle_offset
```


## Functions
```{eval-rst}
.. doxygenfunction:: foc_interrupt
.. doxygenfunction:: enable_foc_loop
.. doxygenfunction:: disable_foc_loop
```