#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Arduino.h"
#include "Actuator.h"

//#define SerialUSB Serial


class Controller
{
    public:
        #define MOTORS_TOT 6

        Controller(const uint8_t IN1_pin[],const uint8_t IN2_pin[],const uint8_t PWM_pin[],const uint8_t POT_pin[]);
        void initTunings(const uint8_t& sampleTime, const float kp[], const float ki[], const float kd[], const float kff[]);
        void reset_integrator();

        void calcNewOutput();  // Calculates a new output
        void actuate();
        void actuate(const float& output);

        void read_PV();

        void set_ref_vel(const float ref_vel[]);
        void set_ref_pos(const float ref_pos[]);
        float get_sensorInput(const uint8_t& actuatorNum);
        float get_prevInput(const uint8_t& actuatorNum);
        float get_sampletime_sec();

    private:
        float kp[MOTORS_TOT];
        float ki[MOTORS_TOT];
        float kd[MOTORS_TOT];
        float kff[MOTORS_TOT];

        float i_sum[MOTORS_TOT];
        float output[MOTORS_TOT];

        Actuator act[MOTORS_TOT];       // Array of six linear-actuator objects

        // Calibrated potentiometer readings at +- 5% from zero/end stroke (5mm and 95 mm)
        const uint16_t potZero[MOTORS_TOT]  = { 180, 180, 180, 180, 180, 180 };
        const uint16_t potEnd[MOTORS_TOT]   = { 3900, 3900, 3900, 3900, 3900, 3900 };
    
        volatile float ref_vel[MOTORS_TOT];
        volatile float ref_pos[MOTORS_TOT];

        float sensorInput[MOTORS_TOT];
        float prevInput[MOTORS_TOT];

        float sampletime_sec;
};


#endif
