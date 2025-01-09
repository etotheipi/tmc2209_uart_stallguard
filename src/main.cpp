/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper motor.
 * 
 * This example code taken from 
 * https://github.com/teemuatlut/TMCStepper/blob/master/examples/TMC_AccelStepper/TMC_AccelStepper.ino
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <AccelStepper.h>
#include <TMCStepper.h>

#define PIN_EN           D10  // Enable
#define PIN_DIR          D9   // Direction
#define PIN_STEP         D8   // Step
#define PIN_DIAG         D2  // Diagnostic/Stall
#define PIN_INDEX        D5  // Index, pulse ever 4 full steps
#define PIN_SW_TX        D6  // UART TX
#define PIN_SW_RX        D7  // UART RX
#define PIN_BUTTON_FWD   D3
#define PIN_BUTTON_REV   D4


#define SEEEDUINO_ESP32_C3

#ifdef SEEEDUINO_ESP32_C3
  // This is needed for the ESP32-C3 version, a software-defined "hardware serial"
  HardwareSerial SerialUART0(0);   // Weird software-defined hardware serial?!
  #define SERIAL_PORT SerialUART0  // TMC2209/TMC2224 HardwareSerial port
#else
  #define SERIAL_PORT Serial1  // TMC2209/TMC2224 HardwareSerial port
#endif

#define MS_PIN_ADDRESS 0b00  // Hardwire MS1 and MS2 to GND
#define R_SENSE 0.11f 

// Use this to drive the TMC2209 via UART (instead of step/dir)
TMC2209Stepper driver = TMC2209Stepper(&SERIAL_PORT, R_SENSE, MS_PIN_ADDRESS);

// I switched to using TMC's HW pulse generator, so don't need AccelStepper library
//AccelStepper stepper = AccelStepper(stepper.DRIVER, PIN_STEP, PIN_DIR);


// MOTION CONSTANTS
constexpr bool     use_stealth_chop = false;
constexpr uint32_t MAX_CURRENT_RMS_mA = 1200;
constexpr float    fullsteps_per_mm = 3.333;  // 30t * 2mm/t = 60 mm/rev, 200 fullsetps/rev * (1 rev / 60 mm)
constexpr uint32_t usteps_per_fullstep = 16;
constexpr uint32_t usteps_per_mm = (uint32_t)(usteps_per_fullstep * fullsteps_per_mm);
constexpr float    accel_mm_per_sec2 = 600.0f;

// Max-max appears to be about (400 rpm) when using stealth chop.  I got up to 2500 rpm without it (@24V)
constexpr float    max_speed_mm_per_sec = 2200.0f;

// MOTION VARIABLES
volatile float target_mm_per_sec = 0.0f;
volatile float current_mm_per_sec = 0.0f;
volatile float dir_sign = 1.0f;  // change to -1 to reverse directions
uint32_t accel_step_interval_ms = 100;
float    accel_step_dv_mm_per_sec = accel_mm_per_sec2 * (float)accel_step_interval_ms / 1000.0f;
uint32_t t_last_accel_update_ms = 0;


// STALL VARIABLES
constexpr uint32_t stallguard_threshold = 20;
volatile bool is_stalled = false;
volatile uint32_t t_last_stall_ms = 0;
const uint32_t t_min_stall_interval_ms = 4000;
uint32_t stall_timeout_ms = 2000;
volatile uint32_t stall_until_ms = 0;

// INDEX/RPM TRACKING
const uint32_t PULSE_BUF_SZ = 8;
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
  // Measuring the RPM via the INDEX pin
  index_ringbuf_n += 1;
  index_pin_pulses[index_ringbuf_n % PULSE_BUF_SZ] = micros();
}


int32_t set_VACTUAL_mm_per_sec(TMC2209Stepper& tmc_driver, float new_mm_per_sec)
{
  int32_t new_vactual = (int32_t)((float)usteps_per_mm * new_mm_per_sec / 0.715f);
  tmc_driver.VACTUAL(new_vactual);
  //Serial.print("New target mm/s: ");
  //Serial.print(new_mm_per_sec);
  //Serial.print(" // Setting VACTUAL: ");
  //Serial.println(new_vactual);
  return new_vactual;
}

float measure_rpm(void)
{
  // Won't reliably measure sub-10RPM (ish) speeds
  uint64_t t_last_us = index_pin_pulses[index_ringbuf_n % PULSE_BUF_SZ];
  uint64_t t_first_us = index_pin_pulses[(index_ringbuf_n+1) % PULSE_BUF_SZ];
  float usec_per_pulse = (float)(t_last_us - t_first_us) / (float)(PULSE_BUF_SZ -1);
  float pulse_per_sec = 1000000.0f / usec_per_pulse;
  // I expected to 50.0f here, but in the end had to figure it out just by testing it.
  float rpm = pulse_per_sec / 26.55f;
  return rpm;
}


void setup()
{
    //SPI.begin();
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Start...");

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
    }
    else
    {
      Serial.println("Using SpreadCycle");
      driver.pwm_autoscale(0);
      driver.en_spreadCycle(1);   
    }

    // It was mentioned in a thread somewhere, to use this for max torque
    if(!use_stealth_chop)
    {
      driver.intpol(true);  //interpolate
      driver.TCOOLTHRS(0);  //disable coolstep (but I think it disables the DIAG pin)
      driver.ihold(0); 
    }

    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.shaft(false);  // True to reverse motor direction
    driver.SGTHRS(stallguard_threshold);

    //stepper.setMaxSpeed(target_usteps_per_sec);
    //stepper.setAcceleration(accel_mm_per_sec2);
    //stepper.setEnablePin(PIN_EN);
    //stepper.setPinsInverted(false, false, true);
    //stepper.enableOutputs();

    attachInterrupt(digitalPinToInterrupt(PIN_DIAG), stall_diag_callback, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_INDEX), index_pulse_callback, RISING);

    set_VACTUAL_mm_per_sec(driver, 0);
}


uint32_t t_curr_ms = 0;
void loop()
{
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

    //stepper.runSpeed();

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
        Serial.print(measure_rpm(), 1);

        // Stallguard load value
        Serial.print(" // SG_RESULT: ");
        Serial.print(driver.SG_RESULT(), DEC);

        Serial.println("");

        set_VACTUAL_mm_per_sec(driver, current_mm_per_sec);
        t_last_accel_update_ms = t_curr_ms;
      }

    }
}
