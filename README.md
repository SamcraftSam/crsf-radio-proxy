# crsf-radio-proxy
RP2040-based firmware for FPV drone remote controller, that acts like a bridge between computer and CRSF-based radios(ELRS).


# Arch notes:

IN -> ELRS Telem   -> USB TX
IN -> USB commands -> ELRS TX

OUT -> ELRS TX
OUT -> USB TX

Everything runs on a Core 0
All the IN and OUT have their own DMAs

Highest priority is for CRSF TX routine. According to the info researched, ELRS TX sends a "hearbeat" that can be used to send channels frame in time.

CRSF RX DMA is being checked in while(1) when TX is not active, checks for the input. If sync -> TX from TX buffer. If telem -> parse and store in RX buf.

USB RX DMA stores recent USB commands that will be put in separate buffer, parsed and processed by control unit. Curently I think thatit should generate response via USB TX for computer to confirm proper reception of the command

USB TX DMA sends telemetry or/and confirmation packets. 

Core 1: PID loops (TODO: description)

Needed to turn simple commands in CRSF-ready values.

Also needed for more advanced stuff (altitude controls).

# Protocol for USB

Must contain commands such as:

- Yaw/Pitch/Roll values.
- Arm/disarm
- Settings for FC (flight mode, VTX pwr)
- Settings for ELRS TX (power, packet rate, telemetry ratio)

