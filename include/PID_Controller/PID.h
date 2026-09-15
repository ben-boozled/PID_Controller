#pragma once

typedef struct {
    /* Controller gains */
    double Kp, Ki, Kd;

    /* Internal state */
    double prevError;           // e[n-1] 
    double prevMeasurement;     // PV[n-1]
    double prevIntegral;        // ui[n-1]
    double prevDerivOut;        // ud[n-1]

    /* Time, seconds */
    double Ts;                  // sampling time 
    double Tf;                  // filter time constants, 1/Tf is the cutoff frequency in rad/s

    /* Constants for derivative */
    double alpha;
    double beta;

    /* Actuator's command signal limits for anti-windup 
    (more conservative than the physical actuator limits) */
    double outputMin;
    double outputMax;
} PID;

/** -------------------------------------------------------------------------- 
 * @brief Initialise the PID controller. Units for outputMin and outputMax will
 *        be the output units of the value returned by PID_update().
 * 
 * @param pid Address of a PID typedef
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param Ts Sampling time, seconds
 * @param Tf Filter time constant, seconds
 * @param outputMin Lower limit of actuator control signal (based on the actuator's physical limits)
 * @param outputMax Upper limit of actuator control signal (based on the actuator's physical limits)
 * 
 * @return None
-------------------------------------------------------------------------- **/
void PID_init(PID* pid, double Kp, double Ki, double Kd, 
              double Ts, double Tf, double outputMin, double outputMax);

/** -------------------------------------------------------------------------- 
 * @brief Preloads the PID controller's integral value instead of starting from
 *        0. Optional function that should be called after PID_init().
 * 
 * @param pid Address of a PID typedef
 * @param preload The preloaded value for the integral
 * 
 * @return None
-------------------------------------------------------------------------- **/
void PID_integralPreload(PID* pid, double preload);

/** --------------------------------------------------------------------------
 * @brief Calculates controller output for the current time step and updates
 *        internal controller state for the next time step. The controller updates
 *        with protections against integral saturation and derivative "kicks".  
 * 
 * @param pid Address of a PID typedef
 * @param setpoint Target setpoint of the process variable at the current time step
 * @param measurement Measured value of the process variable at the current time step
 * 
 * @return output control signal from PID controller
-------------------------------------------------------------------------- **/
double PID_update(PID* pid, double setpoint, double measurement);