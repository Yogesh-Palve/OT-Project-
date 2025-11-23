/*
 * Constants.h
 *
 * 
 * Author: Ritesh
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#endif /* CONSTANTS_H_ */

//PSO Core Parameters
const int SWARM_SIZE = 100; //Number of particles in Swarm
int NO_OF_ITERS = 500;      //Number of Iterations
double WMIN = 0.2;
double WMAX = 0.7;           //Inertial Weight
double C1 = 0.20, C2 = 0.60; //acceleration coefficients
// CASE 3: Bottom-right to top-left curved path
const double START_X = 1200.0;
const double START_Y = -800.0;
const double DESTINATION_X = -1200.0;
const double DESTINATION_Y = 900.0;




const double LOWER_BOUNDARY = -1500.0; //Search Space Lower bound
const double UPPER_BOUNDARY = 1500.0;  //Search Space Upper bound

const double V_MAX = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 10; //Max Particle Velocity
const double POS_MULTIPLE = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 2;
const double VEL_MULTIPLE = (UPPER_BOUNDARY - LOWER_BOUNDARY) / 20;

const int NUM_OBSTACLE = 10;
const double R = 200.0;

const double TARGET_TOLERANCE = 150.0;                  //Tolerance for convergence
const double LOCAL_CONV_TOLERANCE = (NO_OF_ITERS / 20); //Tolerance for local minima convergence
