/**
 * Author: Alan Reiner
 *
 * Adapted from the following example code:
 * https://github.com/teemuatlut/TMCStepper/blob/master/examples/TMC_AccelStepper/TMC_AccelStepper.ino
 * 
 * This code fully utilizes UART to communicate with a standalone TMC2209 stepper driver
 * to drive a NEMA17 stepper motor.  This was a personal exploration to unlock all the 
 * main features of the TMC2209 on a standalone breadboard, not in a 3d printer controller.
 * This was developed in VSCode + PlatformIO.
 * 
 *   - Connects via UART
 *   - Sets parameters (max current, microsteps, etc) via UART
 *   - Flag to switch between StealthChop (quiet) and SpreadCycle (fast)
 *   - Sets up StallGuard and interrupt for the DIAG pin (difficult to calibrate, though)
 *   - driver.VACTUAL(speed) to use hardware pulse generator for constant speed rotation
 *   - Implements acceleration, by capping how much VACTUAL can change every 0.2s
 *   - Implements approx RPM reading via callbacks on INDEX pin (detect slipping?)
 *   - Implemented HardwareSerial.h for Seeeduino XIAO ESP32-C3 to work
 *   - AccelStepper code was originally used before VACTUAL implementation.  Code remains
 *     but is commented out.  Uncomment to do precise, position-based movement
 * 
 * Steppers are intended more for precise movement, not constant RPM like
 * DC motors.  However, this code was more intended to test sensorless homing:
 * I want to run at fixed speed until the platform hits something and stalls.
 * So, this uses the hardware pulse generator (driver.VACTUAL()) to run at
 * constant speed until a stall is detected, then it pauses for 2 seconds.
 * It assumes two input buttons, one to go forward (FWD), the other to go the
 * reverse dir (REV)
 * 
 * AccelStepper code is included (and commented out) which can be used for
 * actual precision stepping, with acceleration.  This was tested before 
 * it was removed and replaced by VACTUAL() stuff.  There's also nothing 
 * stopping you from using the Step/Dir pins like a barbarian.
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TMCStepper.h>
//#include <AccelStepper.h>


//#define USE_MQTT_STATS
#ifdef USE_MQTT_STATS
/**
 * I run most projects with an ESP32 chip (Seeeduino XIAO ESP32-C3 or -S3)
 * and publish running stats to an MQTT topic for easy viewing in Grafana,
 * instead of a bunch of terminal messages.  Totally unnecessary for most
 * people so just make sure "#define USE_MQTT_STATS" is commented out, above.
 */

#include "EspMQTTClient.h"

const char* topicPubSgVal     = "MOTORDATA/stallguard_value";
const char* topicPubRpm       = "MOTORDATA/motor_rpm";
const char* topicPubCurrSpeed = "MOTORDATA/current_speed_mmps";

// To avoid hardcoding WiFi creds in git-committed files, I 
// put the info into a separate .h file that is not pushed.  
#include "../network_params.h"

// Here's what goes into network_params.h, you can hardcode here
#ifndef NETWORK_DETAILS_H
#define NETWORK_DETAILS_H
#define WIFI_SSID ""
#define WIFI_PASS ""
#define MQTT_HOST ""
#define MQTT_PORT 8888
#define MQTT_USER ""
#define MQTT_PASS ""
#endif

EspMQTTClient espClient(
   WIFI_SSID,
   WIFI_PASS,
   MQTT_HOST,
   MQTT_USER,   // MQTT username
   MQTT_PASS,
   "motor_data_client",    // Client name that uniquely identify your device
   MQTT_PORT
);

//void onCommandMsgReceived(const String& message)
void onConnectionEstablished()
{
   Serial.println("MQTT Connection Established.");
   //espClient.subscribe(topicCmd, onCommandMsgReceived);
}
#endif


#define PIN_EN           D10  // Enable
#define PIN_DIR          D9   // Direction
#define PIN_STEP         D8   // Step
#define PIN_DIAG         D2   // Diagnostic/Stall
#define PIN_INDEX        D5   // Index, pulse ever 4 full steps
#define PIN_SW_TX        D6   // UART TX - with 1k resistor to PDN_UART (TMC2209 #4)
#define PIN_SW_RX        D7   // UART RX - direct connection to PDN_UART (TMC2209 #4)
#define PIN_BUTTON_FWD   D3
#define PIN_BUTTON_REV   D4

//#define USE_DEFAULT_UART
#define SEEEDUINO_ESP32_C3
#ifdef SEEEDUINO_ESP32_C3
  // This is needed for the ESP32-C3 version, a software-defined "hardware serial"
  HardwareSerial SerialUART0(0);   // Weird software-defined hardware serial?!
  #define SERIAL_PORT SerialUART0  // TMC2209/TMC2224 HardwareSerial port
#elif defined(USE_DEFAULT_UART)
  // This seems to be the default RX/TX UART Serial object on most devices.
  // Or use SoftwareSerial.h to make your own serial port below
  #define SERIAL_PORT Serial1  
#else
  #include <SoftwareSerial.h>
  bool invert_uart_pins = false;
  SoftwareSerial swSerial(PIN_SW_RX, PIN_SW_TX, invert_uart_pins);
  #define SERIAL_PORT swSerial  
#endif

#define MS_PIN_ADDRESS 0b00  // Hardwire MS1 and MS2 to GND results in 0b00 addr
#define R_SENSE 0.11f 

// Use this to drive the TMC2209 via UART (instead of step/dir)
TMC2209Stepper driver = TMC2209Stepper(&SERIAL_PORT, R_SENSE, MS_PIN_ADDRESS);

// I switched to using TMC's HW pulse generator, so don't need AccelStepper library
//AccelStepper stepper = AccelStepper(stepper.DRIVER, PIN_STEP, PIN_DIR);


/////////////////////////////////////////
// Set to true for quiet (max RPM ~400), false for fast and max torque (~2000 RPM max)
constexpr bool use_stealth_chop = true;

// MOTION CONSTANTS
constexpr uint32_t MAX_CURRENT_RMS_mA = 1200;
constexpr float    fullsteps_per_mm = 3.333f;  // 30t * 2mm/t = 60 mm/rev, 200 fullsetps/rev * (1 rev / 60 mm)
constexpr uint32_t usteps_per_fullstep = 16;
constexpr uint32_t usteps_per_mm = (uint32_t)(usteps_per_fullstep * fullsteps_per_mm);
constexpr float    accel_mm_per_sec2 = 600.0f;

// Based on 3.333 fullsteps_per_mm, ~1mm/s == 1 RPM
constexpr float    max_speed_mm_per_sec = 300.0f;

// MOTION VARIABLES
volatile float target_mm_per_sec = 0.0f;
volatile float current_mm_per_sec = 0.0f;
volatile float dir_sign = 1.0f;  // change to -1 to reverse directions
uint32_t accel_step_interval_ms = 100;
float    accel_step_dv_mm_per_sec = accel_mm_per_sec2 * (float)accel_step_interval_ms / 1000.0f;
uint32_t t_last_accel_update_ms = 0;

// STALL VARIABLES
constexpr uint32_t stallguard_threshold = 20;  // This is the key value to calibrate for stall-detection
constexpr uint32_t t_min_stall_interval_ms = 2000;
constexpr uint32_t stall_timeout_ms = 2000;
volatile bool is_stalled = false;
volatile uint32_t t_last_stall_ms = 0;
volatile uint32_t stall_until_ms = 0;

// INDEX/RPM TRACKING
const uint32_t PULSE_BUF_SZ = 32;
volatile uint32_t index_pin_pulses[PULSE_BUF_SZ];
volatile uint32_t index_ringbuf_n = 0;


void stall_diag_callback(void)
{
  // Don't trigger more than once ever 2sec
  if(millis() - t_last_stall_ms > t_min_stall_interval_ms)
  {
    Serial.println("stall_diag_callback interrupt called");
    is_stalled = true;
    t_last_stall_ms = millis();
  }
}

void index_pulse_callback(void)
{
  // Called every time there is a pulse on the INDEX pin.  Keeps a ring buffer
  // of the microseconds-since-startup of every pulse, for RPM calculation.
  // The docs claim that it pulses every 4 fullsteps, but through experimentation
  // it seems more like ~8 (ish).  See approximate_rpm() for the exact calc.
  index_ringbuf_n += 1;
  index_pin_pulses[index_ringbuf_n % PULSE_BUF_SZ] = micros();
}


int32_t set_VACTUAL_mm_per_sec(TMC2209Stepper& tmc_driver, float new_mm_per_sec)
{
  // The 0.715 is in the TMC2209 docs
  int32_t new_vactual = (int32_t)((float)usteps_per_mm * new_mm_per_sec / 0.715f);
  tmc_driver.VACTUAL(new_vactual);
  //Serial.print("New target mm/s: ");
  //Serial.print(new_mm_per_sec);
  //Serial.print(" // Setting VACTUAL: ");
  //Serial.println(new_vactual);
  return new_vactual;
}

float approximate_rpm(void)
{
  // We approximate RPM via a buffer of N pulse times, in microseconds. 
  // Take the current micros(), minus time of the oldest pulse, divide by N
  // This is not exact, but it gets us a reasonable estimate for RPMs >10 (ish)
  uint64_t t_curr_us = micros();
  uint64_t t_first_us = index_pin_pulses[(index_ringbuf_n+1) % PULSE_BUF_SZ];

  float usec_per_pulse = (float)(t_curr_us - t_first_us) / (float)(PULSE_BUF_SZ);
  float pulse_per_sec = 1000000.0f / usec_per_pulse;
  // I expected to 50.0f here, but in the end had to figure it out just by testing it.
  float rpm = pulse_per_sec / 26.55f;
  return rpm;
}


void setup()
{
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Start...");
    //espClient.enableDebuggingMessages(false);

    SERIAL_PORT.begin(115200, SERIAL_8N1);

    pinMode(PIN_BUTTON_FWD, INPUT_PULLUP);
    pinMode(PIN_BUTTON_REV, INPUT_PULLUP);
    pinMode(PIN_INDEX, INPUT);
    pinMode(PIN_DIAG, INPUT);
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_EN, OUTPUT);

    driver.begin();             // Initiate pins and registeries
    driver.toff(4);
    driver.blank_time(24);
    driver.index_step(true);    // Turn on step output, pulses every 4 fullsteps
    driver.rms_current(MAX_CURRENT_RMS_mA);    // RMS current in mA
    driver.mstep_reg_select(true);  // false to set microsteps by MS1,MS2 pins (no UART)
    driver.microsteps(usteps_per_fullstep);

    if(use_stealth_chop)
    {
      Serial.println("Using StealthChop");
      driver.pwm_autoscale(1);
      driver.en_spreadCycle(0);   
      //driver.en_pwm_mode(1);    // Enable quiet stepping (not for TMC2209)

      driver.TCOOLTHRS(0xFFFFF); // 20bit max
      driver.semin(5);
      driver.semax(2);
      driver.sedn(0b01);
      driver.shaft(false);  // True to reverse motor direction
      driver.SGTHRS(stallguard_threshold);
    }
    else
    {
      Serial.println("Using SpreadCycle");
      driver.pwm_autoscale(0);
      driver.en_spreadCycle(1);   

      // It was mentioned in a thread somewhere, to use this for max torque
      driver.intpol(true);  //interpolate
      driver.TCOOLTHRS(0);  //disable coolstep (but I think it disables the DIAG pin)
      driver.ihold(0); 
    }

    //stepper.setMaxSpeed(target_usteps_per_sec);
    //stepper.setAcceleration(accel_mm_per_sec2);
    //stepper.setEnablePin(PIN_EN);
    //stepper.setPinsInverted(false, false, true);
    //stepper.enableOutputs();

    attachInterrupt(digitalPinToInterrupt(PIN_DIAG), stall_diag_callback, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_INDEX), index_pulse_callback, RISING);

    set_VACTUAL_mm_per_sec(driver, 0);
}


#ifdef USE_MQTT_STATS
uint32_t last_mqtt_update_ms = 0;
char mqttMessage[64];
#endif

uint32_t t_curr_ms = 0;
void loop()
{
    // All "stepper.*" calls are AccelStepper library calls for acceleration-aware 
    // stepping/movement.
    //stepper.runSpeed();

    t_curr_ms = millis();
    if(is_stalled && t_curr_ms > stall_until_ms)
    {
      Serial.println("STALL DETECTED!");
      set_VACTUAL_mm_per_sec(driver, 0);
      target_mm_per_sec = 0;
      current_mm_per_sec = 0;
      is_stalled = false;
      stall_until_ms = t_curr_ms + stall_timeout_ms;

      //stepper.stop();
      //stepper.setCurrentPosition(0);
      //stepper.disableOutputs();
      //stepper.enableOutputs();
    }
    else if(t_curr_ms > stall_until_ms)
    {
      bool btn_fwd = digitalRead(PIN_BUTTON_FWD) == LOW;
      bool btn_rev = digitalRead(PIN_BUTTON_REV) == LOW;

      if(btn_fwd && !btn_rev && target_mm_per_sec == 0.0f)
        target_mm_per_sec = dir_sign * max_speed_mm_per_sec;
      else if(!btn_fwd && btn_rev && target_mm_per_sec == 0.0f)
        target_mm_per_sec = -dir_sign * max_speed_mm_per_sec;
      else if(!btn_fwd && !btn_rev)
        target_mm_per_sec = 0;
    }


    if(t_curr_ms - t_last_accel_update_ms > accel_step_interval_ms)
    {
      bool do_update = false;
      if(current_mm_per_sec < target_mm_per_sec)
      {
        do_update = true;
        current_mm_per_sec += accel_step_dv_mm_per_sec;
        if(current_mm_per_sec > target_mm_per_sec)
          current_mm_per_sec = target_mm_per_sec;
      }
      else if(current_mm_per_sec > target_mm_per_sec)
      {
        do_update = true;
        current_mm_per_sec -= accel_step_dv_mm_per_sec;
        if(current_mm_per_sec < target_mm_per_sec)
          current_mm_per_sec = target_mm_per_sec;
      }
      
      if(do_update)
      {
        Serial.print("Curr mm/s: ");
        Serial.print(current_mm_per_sec);

        Serial.print(" // Targ mm/s: ");
        Serial.print(target_mm_per_sec);

        // Measured RPM (approx) based on INDEX pin pulses
        Serial.print(" // RPM: ");
        Serial.print(approximate_rpm(), 1);

        // Stallguard load value
        Serial.print(" // SG_RESULT: ");
        Serial.print(driver.SG_RESULT(), DEC);

        Serial.println("");

        set_VACTUAL_mm_per_sec(driver, current_mm_per_sec);
        t_last_accel_update_ms = t_curr_ms;
      }

    }

#ifdef USE_MQTT_STATS
    espClient.loop();
    if(millis() - last_mqtt_update_ms > 200)
    {
      last_mqtt_update_ms = millis();

      snprintf(mqttMessage, sizeof(mqttMessage), "%d", driver.SG_RESULT());
      espClient.publish(topicPubSgVal, mqttMessage);

      snprintf(mqttMessage, sizeof(mqttMessage), "%.1f", approximate_rpm());
      espClient.publish(topicPubRpm, mqttMessage);

      snprintf(mqttMessage, sizeof(mqttMessage), "%.1f", current_mm_per_sec);
      espClient.publish(topicPubCurrSpeed, mqttMessage);

    }
#endif
}
