# ld2410_radar
LD2410 Library to connect, read, and configure the radar over uart 

Ld2410 radars are human presence sensors that use UART output data.

This library allows the user to configure the radar by changing the gate energy thresholds, toggling standard, and engineering modes, changing baud rates, and all the features available in the datasheet. It also allows the user to read the energy level of each gate in engineering mode.

The examples folder has Arduino, ESP-IDF, and NRF Connect SDK (really just Zephyr except for the UART baudrate changing function).
