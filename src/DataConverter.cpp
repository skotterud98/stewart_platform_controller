#include "DataConverter.h"

float DataConverter::convIn(const Range pos_vel, const uint16_t& input)
{
    float inMax = 255.0;
    float outMin;
    float outMax;

    if (pos_vel == POS) 
    {
        outMin = posMin;
        outMax = posMax;
    }
    else if (pos_vel == VEL)
    {
        outMin = velMin;
        outMax = velMax;
    }
    else return 0.0;

    float output = (float)input * ((outMax - outMin) / inMax) + outMin;
    
    return output;
}


uint16_t DataConverter::convOut(const Range pos_vel, const float& input)
{
    float outMax = 255.0;         // Max value of 8 bit (1 byte) unsigned integer;
    float inMin;
    float inMax;

    if (pos_vel == POS) 
    {
        inMin = posMin;
        inMax = posMax;
    }
    else if (pos_vel == VEL)
    {
        inMin = velMin;
        inMax = velMax;
    }
    else return 0;
    

    float output = (input - inMin) * outMax / (inMax - inMin);

    return (uint16_t)(output + 0.5);
}


float DataConverter::convIn(const uint16_t& input, const uint16_t& in_low, const uint16_t& in_high)
{
    return (float)(input - in_low) * posMax / (float)(in_high - in_low);
}