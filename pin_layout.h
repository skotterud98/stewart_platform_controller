#ifndef PINS_AND_CONSTANTS_H
#define PINS_AND_CONSTANTS_H

#include <Arduino.h>

// Motor driver H-bridge pins (direction)
const uint8_t M1_IN1 = 11;
const uint8_t M1_IN2 = 12;
const uint8_t M2_IN1 = 10;
const uint8_t M2_IN2 = 9;
const uint8_t M3_IN1 = 5;
const uint8_t M3_IN2 = 6;
const uint8_t M4_IN1 = 22;
const uint8_t M4_IN2 = 24;
const uint8_t M5_IN1 = 28;
const uint8_t M5_IN2 = 26;
const uint8_t M6_IN1 = 30;
const uint8_t M6_IN2 = 32;


// Motor driver PWM pins
const uint8_t M1_PWM = 13;
const uint8_t M2_PWM = 8;
const uint8_t M3_PWM = 7;
const uint8_t M4_PWM = 4;
const uint8_t M5_PWM = 3;
const uint8_t M6_PWM = 2;


// Analog inputs
const uint8_t AMP_SENSE = A0;

const uint8_t M1_POT = A1;
const uint8_t M2_POT = A2;
const uint8_t M3_POT = A3;
const uint8_t M4_POT = A4;
const uint8_t M5_POT = A5;
const uint8_t M6_POT = A6;


#endif
