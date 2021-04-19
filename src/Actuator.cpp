#include "Actuator.h"


Actuator::Actuator()
{
    zero_reading = 0;
    end_reading = ADC_RES;

    actuatorDir = STOP;
}


void Actuator::setPins(const uint8_t& IN1_pin,const uint8_t& IN2_pin,const uint8_t& PWM_pin,const uint8_t& POT_pin)
{
    this->IN1_pin = IN1_pin;
    this->IN2_pin = IN2_pin;
    this->PWM_pin = PWM_pin;
    this->POT_pin = POT_pin;
}


void Actuator::setPotCalibration(const uint16_t& zero_reading,const uint16_t& end_reading)
{
    if (zero_reading < 0 || end_reading > ADC_RES) return;

    this->zero_reading = zero_reading;
    this->end_reading = end_reading;
}



void Actuator::move(const float& output)
{
    int8_t sign;
    if (output < 0.0)
    {
        actuatorDir = RETRACT;
        sign = -1;
    }
    else if (output > 0.0)
    {
        actuatorDir = EXTEND;      // Checks if the ouput is below or beyond middle point and sets direction
        sign = 1;
    }
    else actuatorDir = STOP;

    // Mapping output to a suitable PWM value between 0 and 4095 (if 12 bit PWM is used)
    uint16_t pwm_out = (output * sign * (float)PWM_RES) + 0.5;

    //SerialUSB.println(output);
    
    switch (actuatorDir)
    {
        case RETRACT:
            digitalWrite(IN1_pin, HIGH);
            digitalWrite(IN2_pin, LOW);
            analogWrite(PWM_pin, pwm_out);
            break;
            
        case EXTEND:
            digitalWrite(IN1_pin, LOW);
            digitalWrite(IN2_pin, HIGH);
            analogWrite(PWM_pin, pwm_out);
            break;

        case STOP:
            digitalWrite(IN1_pin, LOW);
            digitalWrite(IN2_pin, LOW);
            analogWrite(PWM_pin, LOW);
            break;
          
        default:
            break;
    }
}



float Actuator::readPosition()
{
    static const uint8_t TOT_READINGS = 5;

    float input = 0.0;

    for (uint8_t reading = 0; reading < TOT_READINGS; reading++)
    {
        input += (float)analogRead(POT_pin);
    }

    uint16_t avg_reading = (uint16_t) (input / (float)TOT_READINGS + 0.5);

    
    return convIn(avg_reading, zero_reading, end_reading);
}
