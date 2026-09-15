#include <stdbool.h>
#include "PID_Controller/PID.h"

void PID_init(PID* pid, double Kp, double Ki, double Kd, double Ts, double Tf, 
              double outputMin, double outputMax){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->prevError = 0.0;
    pid->prevMeasurement = 0.0;
    pid->prevIntegral = 0.0;
    pid->prevDerivOut = 0.0;

    pid->Ts = Ts;
    pid->Tf = Tf;

    pid->alpha = (Ts - 2*Tf)/(Ts + 2*Tf);
    pid->beta = 2*Kd/(Ts + 2*Tf);

    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
}

void PID_integralPreload(PID* pid, double preload){
    pid->prevIntegral = preload;
}

double PID_update(PID* pid, double setpoint, double measurement){
    double currentError = setpoint - measurement;   // e[n]

    /* Proportional contribution */
    double up = pid->Kp * currentError;

    /* Integral contribution 
     * -> uiCandidate is used for anti-windup checks to determine final ui */
    double uiCandidate = pid->Ki * pid->Ts * (currentError + pid->prevError)/2 
                         + pid->prevIntegral; 

    /* Derivative contribution 
     * -> Instead of the setpoint, use the process value/measurement to calculate ud. 
     *    This avoids derivative "kicks" from the infinite gradient when setpoint changes.
     * -> Note that de(t)/dt = d(SP - PV)/dt = -dPV/dt  
     *    Therefore: e[t] - e[t-1] = -(PV[t] - PV[t-1]) */
    double ud = - pid->beta * (measurement - pid->prevMeasurement) 
                - (pid->alpha * pid->prevDerivOut);

    /* Anti-windup implementation via conditional integration to prevent integral saturation */
    double testControlOutput = up + uiCandidate + ud;

    // Saturation check (does the control signal exceed the limits of the actuator 
    // command signal AND push the system further into saturation?)
    // If both checks are true, freeze the integral controller 
    if(testControlOutput < pid->outputMin && currentError < 0){uiCandidate = pid->prevIntegral;} 
    if(testControlOutput > pid->outputMax && currentError > 0){uiCandidate = pid->prevIntegral;}
    
    /* Update the controller's internal state for the next call */
    pid->prevError = currentError;
    pid->prevMeasurement = measurement;
    pid->prevIntegral = uiCandidate;
    pid->prevDerivOut = ud;

    /* Return the control output */
    double controlOutput = up + uiCandidate + ud;
    return controlOutput;
}
