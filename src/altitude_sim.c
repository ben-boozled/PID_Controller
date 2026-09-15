#include "PID_Controller/altitude_sim.h"

void drone_init(Drone* drone, double m, double g, double z0, double v0, double Cd){
    drone->m = m;
    drone->g = g;
    drone->z = z0;
    drone->v = v0;
    drone->Cd = Cd;
}

void drone_update(Drone* drone, double Fthrust, double T){
    /* Calculate vertical acceleration
     * -> Assume Fnet = Fthrust - Fweight - Fdrag */
    double z_acc = Fthrust/(drone->m) - drone->g - (drone->Cd * drone->v);

    /* Update drone state */
    drone->z += drone->v * T + 0.5 * z_acc * T * T; // v0*t + (1/2)*a*t^2;
    drone->v += z_acc * T;

    // Checks whether the drone is on the ground
    if(drone->z < 0){
        drone->z = 0;
        if(drone->v < 0) {
            drone->v = 0;}
    }
}