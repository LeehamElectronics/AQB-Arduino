#pragma once
/* This header file stores all of the GPIO pin output and input definitions. */

/******* Pinout Menu for wiring up AQB *********
 *                                             *
 * Here for easy reference                     *
 * steeringServo = 18                          *
 * throttleServo = 19                          *
 * rearBrakeServo = 20                         *
 *                                             *
************************************************/

/* Relay pinsouts */
#define STARTER_MOTOR_RELAY_PIN 16
#define ENGINE_SPARK_PLUG_PIN 17
#define FRONT_LEFT_HEAD_LIGHT_PIN 33
#define FRONT_RIGHT_HEAD_LIGHT_PIN 32
#define REAR_HEAD_LIGHT_PIN 27

/* Servo Pinouts */
#define STEERING_SERVO_PIN 18
#define THROTTLE_SERVO_PIN 19
#define REAR_BRAKE_SERVO_PIN 23

/* Software Serial Pinouts */
#define SS_IN 25
#define SS_OUT 26 // Connect this to other Arduino board for LCD

/* LED Pinouts */
#define RED_LED_RGB_PIN 15

/* Misc */
#define WIFI_TRIGGER_PIN 34 // To activate WiFi connect portal!
