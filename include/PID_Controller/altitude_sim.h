#pragma once

/* 1-dof quadcopter for altitude control*/
typedef struct {
    double m;        // mass, kg
    double g;        // gravitational acceleration, m/s^2
    double z;        // current altitude, m
    double v;        // vertical velocity, m/s
    double Cd;       // drag coefficient
} Drone;

/** -------------------------------------------------------------------------- 
 * @brief Initialise the drone parameters.
 * 
 * @param drone Address of a Drone typedef
 * @param m Drone mass, kg
 * @param g Gravitational acceleration, m/s^2
 * @param z0 Inital altitude, 0
 * @param v0 Inital vertical velocity, m/s
 * @param Cd Linear drag coefficient, kg/s
 *
 * @return None
-------------------------------------------------------------------------- **/
void drone_init(Drone* drone, double m, double g, double z0, double v0, double Cd);

/** -------------------------------------------------------------------------- 
 * @brief Update the drone's altitude for one time step.
 * 
 * @param drone Address of a Drone typedef
 * @param Fthrust Total thrust force from all four propellers, N
 * @param T Time step, s
 * 
 * @return None
-------------------------------------------------------------------------- **/
void drone_update(Drone* drone, double Fthrust, double T);
