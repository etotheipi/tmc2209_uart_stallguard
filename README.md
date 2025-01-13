## Project: tmc2209_uart_stallguard
Functional code for a standalone TMC2209 stepper motor driver, using UART comms and
configuring DIAG/StallGuard.

Originally adapted from the following example code:
https://github.com/teemuatlut/TMCStepper/blob/master/examples/TMC_AccelStepper/TMC_AccelStepper.ino

### Features

This code fully utilizes UART to communicate with a standalone TMC2209 stepper driver to drive a NEMA17 stepper motor.  This was a personal exploration to unlock all the main features of the TMC2209 on a standalone breadboard, not in a 3d printer controller.  This was developed in VSCode + PlatformIO.

  - Connects to the TMC2209 driver via UART serial comms
  - Sets parameters (max current, microsteps, etc) via UART
  - Flag to switch between StealthChop (quiet) and SpreadCycle (fast)
  - Sets up StallGuard and interrupt for the DIAG pin (difficult to calibrate, though)
  - Uses `driver.VACTUAL(speed)` to use hardware pulse generator for constant speed rotation
  - Implements acceleration, by capping how much VACTUAL can change every 0.2s
  - Implements approx RPM reading via callbacks on INDEX pin (detect slipping?)
  - Implemented HardwareSerial.h for Seeeduino XIAO ESP32-C3 to work
  - AccelStepper code was originally used before VACTUAL implementation.  Code remains but is commented out.  Uncomment to do precise, position-based movement
  - ESP32-MQTT comms for watching motors statistics from Grafana.  This helped me calibrate StallGuard, but totally unnecessary.  I setup most Arduino-ish projects with an ESP32 chip and MQTT for monitoring.

Steppers are intended more for precise movement, not constant RPM like DC motors.  However, this code was more intended to test sensorless homing: I want to run at fixed speed until the platform hits something and stalls.  So, this uses the hardware pulse generator (driver.VACTUAL()) to run at constant speed until a stall is detected, then it pauses for 2 seconds.  It assumes two input buttons, one to go forward (FWD), the other to go the reverse dir (REV)

AccelStepper code is included (and commented out) which can be used for actual precision stepping, with acceleration.  This was tested before it was removed and replaced by VACTUAL() stuff.  There's also nothing stopping you from using the Step & Dir pins directly like barbarian.

### Wiring Notes:
- MS1 and MS2 both to GND -- when using UART, this sets address to 0b00 (see `MS_PIN_ADDRESS` below, up to 4x TMC2209 can be used/addressed at once)
- Pin 4 on the TMC2209 (below MS1/MS2) is the PDN_UART pin.  It is connected to BOTH the UART RX and TX pins on the controller:
  - RX:  Direct connection
  - TX:  Connected via 1k resistor
- For pin 4 on the TMC2209 to be **the** correct pin, the docs claim that two tiny pads on the back of the TMC2209 need to be jumped via solder.
  - Docs claim it should come like that, and change it if you want to use pin 5 instead.  Mine came with neither pad jumped
  - I added solder to jump the pads for pin 4.  I don't know if it was actually necessary.
- Recommend 47-100 uF capacitor across VMOT-GND on the driver
- Make sure to connect controller logic +V and GND to the two bottom right pins on the TMC2209.  Handles both 3.3V and 5V (but not 12/24V!)
- The 4 NEMA17 motor wires seem to be random order depending on manufacturer.  But you won't hurt anything if you get it wrong, then simply reorder them.  You can also use multimeter to check continuity to determine which pairs are coils.

     
