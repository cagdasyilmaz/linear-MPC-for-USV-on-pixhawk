#ifndef CONSTANTS_H
#define CONSTANTS_H

/* Time Constants on System */
#define sampling_time 0.2f

/* Controller Parameters */
#define surge_weight 20.0f  // weight term in Q matrix
#define yaw_weight 100.0f    // weight term in Q matrix
#define input_weight 5.0f   // weight term in R matrix
#define N_C 5               // control horizon
#define N_P 15              // output horizon

/* LTV State-Space Parameters */
#define N_I 2               // number of input
#define N_STATE 7           // Number of states
#define N_OUTPUT 2          // Number of output

/* Phyical Limits */
#define U_MAX 40.0f         // Maximum applied torque value
#define U_MIN -40.0f        // Minimum applied torque value
#define DELTA_U_MAX 40.0f   // Maximum change in thruster
#define DELTA_U_MIN -40.0f  // Minimum change in thruster

#endif // CONSTANTS_H
