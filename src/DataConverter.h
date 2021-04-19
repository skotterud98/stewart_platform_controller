#ifndef NUMCONVERTER_H
#define NUMCONVERTER_H

#include <Arduino.h>

class DataConverter
{
    protected:
        enum Range
        {
            POS,
            VEL
        };

        static float convIn(const Range pos_vel, const uint16_t& input);
        static float convIn(const uint16_t& input, const uint16_t& in_low, const uint16_t& in_high);

        static uint16_t convOut(const Range pos_vel, const float& input);

        static constexpr uint16_t PWM_RES       = 4095;
        static constexpr uint16_t ADC_RES       = 4095;

        static constexpr float posMin    = 0.0;
        static constexpr float posMax    = 0.09;
        
        static constexpr float velMin    = -0.06;
        static constexpr float velMax    = 0.06;
};

#endif