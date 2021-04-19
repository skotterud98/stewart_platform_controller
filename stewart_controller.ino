#include "pin_layout.h"
#include "src/Controller.h"
#include "src/CANbus.h"

//#define SerialUSB Serial


/************************************** Static variables & objects *************************************/

// Grouping the pins together in arrays with one column for each actuator/motor-driver
const uint8_t IN1_PINS[MOTORS_TOT]  = { M1_IN1, M2_IN1, M3_IN1, M4_IN1, M5_IN1, M6_IN1 };
const uint8_t IN2_PINS[MOTORS_TOT]  = { M1_IN2, M2_IN2, M3_IN2, M4_IN2, M5_IN2, M6_IN2 };
const uint8_t PWM_PINS[MOTORS_TOT]  = { M1_PWM, M2_PWM, M3_PWM, M4_PWM, M5_PWM, M6_PWM };
const uint8_t POT_PINS[MOTORS_TOT]  = { M1_POT, M2_POT, M3_POT, M4_POT, M5_POT, M6_POT };

// Arrays of gains to the controller
const float kp[MOTORS_TOT]     = { 75, 75, 75, 75, 75, 75 };    // Proportional gain
const float ki[MOTORS_TOT]     = { 0, 0, 0, 0, 0, 0 };    // Integral gain
const float kd[MOTORS_TOT]     = { 0, 0, 0, 0, 0, 0 };    // Derivative gain
const float kff[MOTORS_TOT]    = { 30.92, 31.65, 31.51, 30.66, 32.40, 31.74 };    // Feedforward gain    

// Main controller object which includes the regulator alghorithm and six actuator objects.
// Actuator/motor-driver pins as parameters for constructor to initialize actuator objects
Controller control(IN1_PINS, IN2_PINS, PWM_PINS, POT_PINS);

// CAN-bus handler which deals with communication and keep track of what state the platform is in
CANbus can(&control);

// Sampling variables
const uint16_t SAMPLETIME = 10000;  // [ms]
uint32_t currentTime = 0;
uint32_t lastTime = 0;

// Variables to keep track of CAN input error
uint8_t err_count = 0;
bool checkError = false;


/********************************************** Main ***************************************************/

/*
*   Init-function
*/
void setup()
{
    SerialUSB.begin(115200);

    analogReadResolution(12);
    analogWriteResolution(12);

    for (int num = 0; num < MOTORS_TOT; num++)
    {
        pinMode(IN1_PINS[num], OUTPUT);
        pinMode(IN2_PINS[num], OUTPUT);
        pinMode(PWM_PINS[num], OUTPUT);
    }

    control.initTunings(SAMPLETIME, kp, ki, kd, kff);

    Can0.begin(500000);
    Can0.attachObj(&can);

    Can0.setRXFilter(SETPOINT_MB, 0xA0, 0x7FC, false);        // syntax: mailbox, id, mask, extended
    can.setCallback(SETPOINT_MB);
}


/*
*   Endless loop-function
*/
void loop() 
{
    /*
    if (can.run())
    {
        control.read_PV();
        control.reset_integrator();
        //SerialUSB.println("START!");

        while(CANflush());
        can.reset();
        can.setCallback(SETPOINT_MB);
        can.reply(START);

        err_count = 0;
    }


    if (can.stop())
    {
        control.actuate(0);
        //SerialUSB.println("STOP");
        
        can.removeCallback(SETPOINT_MB);
        while(CANflush());
        can.reset();
        can.reply(STOP);
    }
    */

    // Store current number of milliseconds since program startup
    currentTime = micros();

    if (currentTime - lastTime >= SAMPLETIME && can.received())
    {
        //SerialUSB.println("SAMPLING OK!");
        lastTime = currentTime;
        err_count = 0;

        can.removeCallback(SETPOINT_MB);

        // Reads each actuators position (PV - process variable)
        control.read_PV();

        // Computes a new output and direction of each actuator, based on given setpoint
        // and sensor feedback. Includes a PID controller with velocity feedforward.
        control.calcNewOutput();

        can.setCallback(SETPOINT_MB);

        // Move all actuators using this new calculated output
        control.actuate();

        // Replies to CAN-bus with feedback position and velocity data
        can.reply();

        can.reset();

        // Set checkError variable to true, to start tracking sampling time of CAN input
        if(!checkError) { checkError = true; }

    }
    else if (currentTime - lastTime >= SAMPLETIME*1.1 && checkError)
    {
        lastTime = currentTime;
        err_count++;
        can.reset();

        SerialUSB.println("###########");

        if (err_count > 3) 
        {
            SerialUSB.println("ERROR");
            control.actuate(0);
            control.reset_integrator();
            
            can.removeCallback(SETPOINT_MB);
            while(CANflush());
            can.reset();
            can.setCallback(SETPOINT_MB);

            checkError = false;
            err_count = 0;
        }
    }
}

/*-------------------------------------------- End of Main ---------------------------------------------------*/





/*************************************** Static function definitions **********************************************/

bool CANflush()
{
    static CAN_FRAME rxFlush;

    if (Can0.rx_avail()) 
    {
        Can0.get_rx_buff(rxFlush);
        return true;
    }
    
    return false;
}
