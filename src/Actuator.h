#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include "DataConverter.h"


class Actuator: private DataConverter
{
    public:

        enum Direction
        {   
            EXTEND, 
            RETRACT, 
            STOP 
        };

        Actuator();
        void setPins(const uint8_t& IN1_pin, const uint8_t& IN2_pin, const uint8_t& PWM_pin, const uint8_t& POT_pin);
        void setPotCalibration(const uint16_t& zeroReading, const uint16_t& endReading);

        void move(const float& output);         // Move actuator in specified direction with a set output from the regulator
        
        float readPosition();

    private:
    
        uint8_t IN1_pin;
        uint8_t IN2_pin;
        uint8_t PWM_pin;
        uint8_t POT_pin;

        Direction actuatorDir;

        uint16_t zero_reading;
        uint16_t end_reading;
};

#endif
