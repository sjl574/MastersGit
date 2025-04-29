/*
  This file has been made to contain all Project default / constant values such as pin numbers and physical values
  Aim is to reduce code in main script
*/
#include <Arduino.h>

/*      Set all motor defaults and constants here     */
//Step constants
//Whole steps per revolution
#define UPPER_MOTOR_SPR 200       
#define LOWER_MOTOR_SPR 200 
//Microsteps set on controll DIP switches
#define UPPER_MICROSTEPS 4
#define LOWER_MICROSTEPS 8
//GEARING RATIO
#define UPPER_GR 2
#define LOWER_GR 3
//Correction factors
#define UPPER_CF 1.125
#define LOWER_CF 0.72
//Speed 
#define UPPER_MAX_RPM 100
#define LOWER_MAX_RPM 75
#define UPPER_DEFAULT_RPM 40
#define LOWER_DEFAULT_RPM 40
//Acceleration
#define UPPER_MAX_ACCEL 50
#define LOWER_MAX_ACCEL 50
#define UPPER_DEFAULT_ACCEL 50
#define LOWER_DEFAULT_ACCEL 40
//Pins
#define UPPER_DIR_PIN 13
#define UPPER_STEP_PIN 12
#define LOWER_DIR_PIN 11
#define LOWER_STEP_PIN 10
//Clearnace distance
#define LOWER_CLEARANCE_DEG 20
#define UPPER_CLEARANCE_DEG 10
#define LOWER_BOOT_DEG 90
#define UPPER_BOOT_DEG 45
//Axis Flip (set to -1 for flip or 1 for not flip)
#define UPPER_AXIS_FLIP -1
#define LOWER_AXIS_FLIP 1



/*Beam break sensor defaults*/
//Interrupt pins for beam breaks
#define UPPER_BB_PIN 2
#define LOWER_BB_PIN 3
//num of sensors
#define NUM_BB_SENSORS 2

/*Firing Mechanism Defaults*/
#define FIRING_SERVO_PIN 4
#define FIRING_MOTOR_PIN 5



/*Serial COMMAND Code*/
//command value to function
#define CMD_MOTOR_UPPER   0x01
#define CMD_MOTOR_LOWER   0x02
#define CMD_FIRE      0x03
#define CMD_GET_LIDAR 0x04
#define CMD_MOTOR_UPPER_OOB 0x05
#define CMD_MOTOR_LOWER_OOB 0x06


//---DECIMAL SHIFT VALUES - shift passed serial val into float e.g.(deg to turn = serial value * shift)
// shift values are for conversion from serial input to actual value so inverse should be applied in opposition
//MOTOR DEG
#define DEG_DECIMAL_SHIFT 0.01


//message byte lengths
#define SERIAL_CMD_LEN_BYTES 4
#define SERIAL_VAL_LEN_BYTES 4 


