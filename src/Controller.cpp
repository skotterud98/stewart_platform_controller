#include "Controller.h"


Controller::Controller(const uint8_t IN1_pin[], const uint8_t IN2_pin[], const uint8_t PWM_pin[], const uint8_t POT_pin[])
{
    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        act[num].setPins(IN1_pin[num], IN2_pin[num], PWM_pin[num], POT_pin[num]);
        act[num].setPotCalibration(potZero[num], potEnd[num]);

        i_sum[num]      = 0;
        prevInput[num]  = 0;
        output[num]     = 0;
        
        kp[num]         = 0;
        ki[num]         = 0;
        kd[num]         = 0;
        kff[num]        = 0;
    }
}


void Controller::initTunings(const uint8_t& sampletime, const float kp[], const float ki[], const float kd[], const float kff[])
{
    if (sampletime > 0)
    {
        for (uint8_t num = 0; num < MOTORS_TOT; num++)
        {
            if (kp[num] < 0 || ki[num] < 0 || kd[num] < 0 || kff[num] < 0) return;
        }

        this->sampletime_sec = ((float)sampletime) / 1000.0;
        
        for (uint8_t num = 0; num < MOTORS_TOT; num++)
        {
            this->kp[num]   = kp[num];
            this->ki[num]   = ki[num] * sampletime_sec;
            this->kd[num]   = kd[num] / sampletime_sec;
            this->kff[num]  = kff[num];
        }
    }
}


void Controller::read_PV()
{
    for (uint8_t num = 0; num < MOTORS_TOT; num ++)
    {
        this->sensorInput[num] = act[num].readPosition();
    }
}


void Controller::calcNewOutput()
{
    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        SerialUSB.print(ref_vel[num], 10);
        
        // Error between desired and actual input_position
        float input = sensorInput[num];
        float error = ref_pos[num] - input;
        float deltaInput = input - prevInput[num];
        
        SerialUSB.print("\t");
        SerialUSB.print(deltaInput / sampletime_sec, 10);

        // If error is larger than 0.5 mm
        if (true /*error > 0.0005  ||  error < -0.0005*/)
        {
            // Settting output variable equal to the proportional term
            output[num] = kp[num] * error;
            
            
            // Calculating the integral term
            i_sum[num] += ki[num] * error;
            if (i_sum[num] > 1.0) i_sum[num] = 1.0;             // Limiting integral contribution to prevent integral-windup
            else if (i_sum[num] < -1.0) i_sum[num] = -1.0;


            // Adding integral and subtracting derivative on input, instead of error,
            // this is to help preventing output "kicks"
            output[num] += i_sum[num] - kd[num] * deltaInput;


            // Adding the feedforward velocity ref
            output[num] += (kff[num] * ref_vel[num]);
            
            //SerialUSB.print("\t");
            //SerialUSB.println(sensorInput[num], 10);
             
            // Clamp output to be within the max/min values
            if (output[num] < -1.0) output[num] = -1.0;
            else if (output[num] > 1.0) output[num] = 1.0;
        }
        else
        {
            output[num] = 0;
        }

        // Store for next iteration
        prevInput[num] = input;
    }
}


void Controller::reset_integrator()
{
    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        i_sum[num] = 0;
    }
}

/*
*   Move actuator with the stored output.
*/
void Controller::actuate()
{
    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        act[num].move(this->output[num]);
        //SerialUSB.println(this->output[num]);
    }
}



/*
*   Overloading to be able to enter a certain output in open-loop without going through the regulator.
*   This function with manual output also initialize the PID variables to current state.
*/
void Controller::actuate(const float& output)
{
    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        act[num].move(output);
    }
}


void Controller::set_ref_vel(const float ref_vel[])
{
    for (uint8_t num = 0; num < 6; num++)
    {
        this->ref_vel[num] = ref_vel[num];
    }
}

void Controller::set_ref_pos(const float ref_pos[])
{
    for (uint8_t num = 0; num < 6; num++)
    {
        this->ref_pos[num] = ref_pos[num];
    }
}


float Controller::get_sensorInput(const uint8_t& actuatorNum) { return this->sensorInput[actuatorNum]; }

float Controller::get_prevInput(const uint8_t& actuatorNum) { return this->prevInput[actuatorNum]; }

float Controller::get_sampletime_sec() { return this->sampletime_sec; }



/*
void Controller::ramp_to_pos(const float& input_position)
{
    static const float timeToNeutral = 5.0;     // [sec]
    
    float ramp_coeff[MOTORS_TOT];

    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        ramp_coeff[num] = (input_position - sensorInput[num]) / (timeToNeutral / sampletime_sec);
        ref_pos[num] = sensorInput[num];
        ref_vel[num] = 0;
    }

    uint32_t timePrev = 0;

    while(!at_position(input_position))
    {
        uint32_t timeNow = millis();
        
        if(timeNow - timePrev >= (uint8_t)(sampletime_sec * 1000))
        {
            for (uint8_t num = 0; num < MOTORS_TOT; num++)
            {
                ref_pos[num] += ramp_coeff[num];
            }
            
            read_PV();
            calcNewOutput();
            actuate();

            timePrev = timeNow;
        }
    }
}



bool Controller::at_position(const float& input_position)
{
    bool allArrived = true;

    for (uint8_t num = 0; num < MOTORS_TOT; num++)
    {
        allArrived &= (sensorInput[num] >= input_position-0.0005 && sensorInput[num] <= input_position+0.0005);
    }

    return allArrived;
}
*/
