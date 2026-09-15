#include <stdio.h>
#include <math.h>
#include "PID_Controller/PID.h"
#include "PID_Controller/altitude_sim.h"

/* --------------------------------- 
 *User inputs for the PID controller 
 * --------------------------------- */
#define Kp                  2.6      // N/m for output in N
#define Ki                  1.2      // N/(m*s)
#define Kd                  1.4      // (N*s)/m

#define Ts                  0.01     // sample time
#define Tf                  0.05     // filter time constant

#define thrustMin           0        // combined thrust limits of all four propellers, N
#define thrustMax           15       // 

#define preload             0.0      // preload value of the integral term

#define settlingBand        0.03     // usually within 3% or 5% of the final setpoint

/* --------------------------------------- 
 * User inputs for the altitude simulation
 * --------------------------------------- */
#define m                   0.5      // drone mass, kg
#define g                   9.81     // gravitational acceleration, m/s^2
#define z0                  0.0      // initial altitude, m
#define v0                  0.0      // initial velocity, m/s
#define Cd                  0.3      // linear drag coefficient, kg/s

#define setpoint            5        // m
#define duration            15       // simulation duration, s                                     

/* --------------------------------------- 
 * ******* Running the simulation ********
 * --------------------------------------- */
int main(){
    printf("\n----- Start Simulation -----");
   
    /* Initialise variables */ 
    PID pid;
    PID_init(&pid, Kp, Ki, Kd, Ts, Tf, thrustMin, thrustMax);
    PID_integralPreload(&pid, preload);

    Drone drone;
    drone_init(&drone, m, g, z0, v0, Cd);

    /* Simulate step signal: hold 0 m for 1 s, then send the command setpoint */
    double t_hold = 1.0; 
    double z_low = 0.0;

    int t_steps = (int) (duration/Ts);

    /* Store results in CSV (results can be plotted with plot_csv.py)*/
    FILE *csv = fopen("altitude_sim.csv", "w");
    if(!csv) {
        fprintf(stderr, "\nUnable to open output CSV file ._.");
        return 1;
    }

    fprintf(csv, "Time (s),Setpoint (m),Altitude (m),Velocity (m/s),Thrust (N),Error (m),PID Out (N)\n");

    /* Checks for whether the system is in steady state*/
    double errorBand = settlingBand * setpoint;
    double settlingTimestamp = -1.0;
    int currentSettlingCount = 0;
    const int settleHoldThreshold = (int) (0.5/Ts); // must be within the settling error band for 0.5s
    double settlingThrust = -1.0;

    /* Loop through all the time steps */
    for(int i=0; i<=t_steps; i++){
        double t_current = i*Ts;
        double currentSetpoint = (t_current <= t_hold) ? z_low : setpoint;

        double pid_out = PID_update(&pid, currentSetpoint, drone.z);
        double thrust = pid_out;

        // Clamp the maximum and minimum thrust to the actuator limits
        if(thrust > thrustMax) {thrust = thrustMax;}
        if(thrust < thrustMin) {thrust = thrustMin;}

        double error = currentSetpoint - drone.z;

        fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                t_current, currentSetpoint, drone.z, drone.v, thrust, error, pid_out);

        // Increment or reset settling count depending on whether altitude is within settling error band     
        
        if(fabs(drone.z - setpoint) <= errorBand){
            currentSettlingCount++;
        } else {
            currentSettlingCount = 0;
        }

        if (currentSettlingCount == settleHoldThreshold && settlingTimestamp < 0) {
            settlingTimestamp = t_current - 0.5; // time elapsed when first entering the band
            settlingThrust = thrust;
        }
        
        drone_update(&drone, pid_out, Ts);
    }

    fclose(csv);
    printf("\n----- Results written to CSV -----");

    printf("\n----- Summary of Results -----");
    printf("\nSettled within +/-%.2fm of %.2fm (%.2f%% error) at t=%.2fs", errorBand, 
           (double) setpoint, (double) settlingBand*100,settlingTimestamp);
    printf("\nSettling time: %.2fs", settlingTimestamp - t_hold);
    printf("\nThrust at settling time: %.2fN", settlingThrust);
    printf("\nExpected hover thrust: %.2fN", m*g);
    printf("\n----- Simulation Complete -----");

    return 0;
}
