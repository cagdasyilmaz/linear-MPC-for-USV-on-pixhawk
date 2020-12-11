#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <string.h>
#include <math.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>
#include "constants.h"

// Compare Two Vales and Take Larger Number
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
/* Compare Two Vales and Take Smaller Number */
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/*
 * mpc_controller_3dof
 * 1 - This function makes Jacobian substitution
 * 2 - Basic discretization
 * 3 - Optimization problem to find proper torque values
 */
typedef struct mpc_output
{
    matrix::Matrix<float, N_STATE, 1> return_state_vector;
    float delta_left_thrust;
    float delta_right_thrust;
}mpc_output;

mpc_output mpc_controller_3dof(struct vehicle_local_position_s *local_pos,
                        struct vehicle_attitude_s *vehicle_att,
                        float U_L, float U_R,
                        matrix::Matrix<float, N_STATE, 1> prev_state_vector,
                        float desired_surge, float desired_yaw);

/*
 * Static Matricies used in mpc_controller_3dof
 */
matrix::Matrix<float, N_STATE, N_I> Create_B_Matrix();
matrix::Matrix<float, N_OUTPUT, N_STATE> Create_C_Matrix();
matrix::SquareMatrix<float, N_OUTPUT*N_P> Create_Q_Matrix();
matrix::SquareMatrix<float, N_C*N_I> Create_R_Matrix();
matrix::SquareMatrix<float, N_STATE> Create_Eye_For_Disc();
/*
 * Matricies and vectors about constraints
 * constraints : inequality constraint
 * Scenario 2 is handled
 */
matrix::Matrix<float, 2*N_C*N_I, N_C*N_I> Create_A_Constraints();
matrix::Matrix<float, 2*N_C*N_I, 1> Create_b_Constraints();

/* Signum function */
int signum(float value);

#endif // MPC_CONTROLLER_H
