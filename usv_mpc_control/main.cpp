/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file main.cpp
 *
 * Example implementation of a model predictive controller.
 * for unmanned surface vehicle
 *
 * @author Cagdas Yilmaz
 */
#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
//#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
//#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
//#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_reference_states.h>

#include <parameters/param.h>
#include <lib/ecl/geo/geo.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>

/* process-specific header files */
#include "params.h"
#include "constants.h"
#include "guidance.h"
#include "mpc_controller.h"

#define MIXER_LIM 0.4f
#define FILTER_CONSTANT_YAW   1.5f
#define FILTER_CONSTANT_SURGE 0.5f
#define SAMPLING_TIME 0.2f

/* Prototypes */

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int usv_mpc_control_main(int argc, char *argv[]);

struct filtered_references
{
    float filtered_yaw;      // filtered desired yaw angle
    float filtered_surge;    // filtered desired surge speed
};

struct filtered_references lowpass_filter_for_references(float prev_filtered_yaw,
                                                         float desired_yaw,
                                                         float yaw_filter_coeffient,
                                                         float prev_filtered_surge,
                                                         float desired_surge,
                                                         float surge_filter_coeffient);
/**
 * Mainloop of daemon.
 */
int usv_mpc_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);


/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */


/* Main Thread */
int usv_mpc_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;
    bool start_guidance = true;

    /* Initialize time used in the system
     * Firmware/src/drivers/drv_hrt.h
     * uint64_t in usecs
     *
     *  Absolute time, in microsecond units.
     *
     *  Absolute time is measured from some arbitrary epoch shortly after
     *  system startup.  It should never wrap or go backwards.
     */
    hrt_abstime time_loop_start = 0;           // Start time of the loop
    hrt_abstime time_loop_exec = 0;            // Execution time of the loop

    //matrix::Vector<float, 6> prev_state_vector;
    matrix::Matrix<float, N_STATE, 1> prev_state_vector;
    prev_state_vector.setZero();
    prev_state_vector._data[6][0] = 1.0f;

    matrix::Matrix<float, N_I, 1> input_vector;
    input_vector.setZero();

    struct pp_guidance_outpus take_outputs_pp;
    take_outputs_pp.desired_surge_pp = 0.0f;
    take_outputs_pp.desired_yaw_pp = 0.0f;

    struct filtered_references take_filtered_references;
    mpc_output mpc_return;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

    /* Guidance Parameters */
    int yaw_state = 0;
    float psi_last = 0.0f;
    float accumulation = 0.0f;
    int way_point_index = 0;
    int way_point_change = 0;

    float way_point_info[9][5] = {
        { 9.2082,   2.0978, 1, 2, 0.75},  /* initializiers for row indexed by 0 */
        {13.4344,  -6.9653, 1, 2, 0.75},  /* initializiers for row indexed by 1 */
        {21.3810, -13.0509, 1, 2, 0.75},  /* initializiers for row indexed by 2 */
        {24.1176, -16.1165, 1, 1, 0.75},       /* initializiers for row indexed by 3 */
        {25.0596, -18.2580, 1, 1, 0.75},       /* initializiers for row indexed by 4 */
        {25.6589, -20.8891, 1, 1, 0.75},       /* initializiers for row indexed by 5 */
        {26.5297, -23.1323, 1, 1, 0.75},       /* initializiers for row indexed by 7 */
        {28.0149, -24.4981, 1, 1, 0.75},       /* initializiers for row indexed by 8 */
        {30.4718, -26.2197, 1, 1, 0.75},       /* initializiers for row indexed by 9 */
    };
    int number_of_wp = sizeof(way_point_info)/sizeof(*way_point_info);

    /* Thruster Parameters */
    float U_L = 0.0f;
    float U_R = 0.0f;

    /* Filter Coefficient Values */
    float yaw_filter_coeffient = FILTER_CONSTANT_YAW/(FILTER_CONSTANT_YAW + SAMPLING_TIME);
    float surge_filter_coeffient = FILTER_CONSTANT_SURGE/(FILTER_CONSTANT_SURGE + SAMPLING_TIME);

    /* Inital Filter Output Values */
    float yaw_filtered_prev = 0.0f;
    float surge_filtered_prev = 0.5f;

    float surge_speed = 0.0f;
    /* Initial Cost Values */
    float output_error_cost = 0.0f;
    float control_effort_cost = 0.0f;
    float total_cost = 0.0f;

    /*
	 * Declare and safely initialize all structs to zero.
	 *
	 * These structs contain the system state and things
	 * like attitude, position, the current waypoint, etc.
	 */
	struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_local_position_s local_pos;
    memset(&local_pos, 0, sizeof(local_pos));

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

    struct vehicle_reference_states_s reference_states;
    memset(&reference_states, 0, sizeof(reference_states));

	/* publish actuator controls with zero values */
	for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
		actuators.control[i] = 0.0f;
	}

	/*
	 * Advertise that this controller will publish actuator
	 * control values and the rate setpoint
	 */
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

    /* advertise vehicle_reference_states topic */
    orb_advert_t reference_states_pub = orb_advertise(ORB_ID(vehicle_reference_states), &reference_states);

	/* subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    orb_set_interval(att_sub, 200);

    int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	/* Setup of loop */

    struct pollfd fds[1];

    fds[0].fd = att_sub;

    fds[0].events = POLLIN;

    while (!thread_should_exit) {
    //for (int i = 0; i < 40; i++) {

        //time_bet_2lastloopcalls = hrt_elapsed_time(&time_loop_start);
        //time_bet_2loopcalls_max = MAX(time_bet_2loopcalls_max, time_bet_2lastloopcalls);
        time_loop_start = hrt_absolute_time();  // mark the start of while loop

        /*
         * Wait for a sensor, check for exit condition every 500 ms.
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new attitude measurement or
         * a param update is published. So no latency in contrast to the polling
		 * design pattern (do not confuse the poll() system call with polling).
		 *
		 * This design pattern makes the controller also agnostic of the attitude
		 * update speed - it runs as fast as the attitude updates with minimal latency.
		 */
        int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/*
			 * Poll error, this will not really happen in practice,
			 * but its good design practice to make output an error message.
			 */
			warnx("poll error");

		} else if (ret == 0) {
            PX4_INFO("NOTHING CHANCED!!!!");
			/* no return value = nothing changed for 500 ms, ignore */
        } else {

			/* only run controller if attitude changed */
            if (fds[0].revents & POLLIN) {


				/* Check if there is a new position measurement or position setpoint */

                //bool pos_updated;
                //orb_check(global_pos_sub, &pos_updated);
                //orb_check(local_pos_sub, &pos_updated);
                //bool manual_sp_updated;
                //orb_check(manual_sp_sub, &manual_sp_updated);

                bool att_updated;
                //orb_check(global_pos_sub, &pos_updated);
                orb_check(att_sub, &att_updated);


                if(att_updated == true)
                {
                   //PX4_INFO("Updated");
                   /* get a local copy of attitude */
                   orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
                   orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);


                    if (start_guidance == true)
                    {
                        /*
                        * Before starting guidance check the polar coordinate region
                        * This is only done one time at starting
                        */
                        if (sgn(local_pos.y) == true && sgn(local_pos.x) == true)
                        {
                        yaw_state = 1;
                        }

                        else if (sgn(local_pos.y) == false && sgn(local_pos.x) == true)
                        {
                            yaw_state = 2;
                        }
                        else if (sgn(local_pos.y) == false && sgn(local_pos.x) == false)
                        {
                            yaw_state = 3;
                        }
                        else
                        {
                            yaw_state = 4;
                        }

                        /*
                        PX4_INFO("wp_index_pp: \t%d", (int)take_outputs_pp.wp_index_pp);
                        PX4_INFO("finished_pp: \t%d", (bool)take_outputs_pp.finished_pp);
                        PX4_INFO("desired_surge_pp: \t%8.4f", (double)take_outputs_pp.desired_surge_pp);
                        PX4_INFO("desired_yaw_pp: \t%8.4f", (double)take_outputs_pp.desired_yaw_pp);
                        PX4_INFO("psi_last_pp: \t%8.4f", (double)take_outputs_pp.psi_last_pp);
                        PX4_INFO("accumulation_pp: \t%8.4f", (double)take_outputs_pp.accumulation_pp);
                        PX4_INFO("yaw_state_pp: \t%8.4f", (double)take_outputs_pp.yaw_state);
                        */
                        start_guidance = false;
                    }

                    actuators.timestamp = hrt_absolute_time();
                    reference_states.timestamp = actuators.timestamp;

                    reference_states.way_point_x_position = way_point_info[way_point_index][0];
                    reference_states.way_point_y_position = way_point_info[way_point_index][1];


                    take_outputs_pp = pp_guidance(&local_pos, way_point_index,
                                              number_of_wp, yaw_state,
                                              psi_last, accumulation,
                                              way_point_info);


                    take_filtered_references = lowpass_filter_for_references(yaw_filtered_prev,
                                                                             take_outputs_pp.desired_yaw_pp,
                                                                             yaw_filter_coeffient,
                                                                             surge_filtered_prev,
                                                                             take_outputs_pp.desired_surge_pp,
                                                                             surge_filter_coeffient);

                    take_outputs_pp.desired_yaw_pp = take_filtered_references.filtered_yaw;
                    yaw_filtered_prev = take_outputs_pp.desired_yaw_pp;
                    take_outputs_pp.desired_surge_pp = take_filtered_references.filtered_surge;
                    surge_filtered_prev = take_outputs_pp.desired_surge_pp;

                    //PX4_INFO("wp_index_pp: \t%d", (int)take_outputs_pp.wp_index_pp);
                    //PX4_INFO("Desired surge speed: \t%8.8f", (double)take_outputs_pp.desired_surge_pp);
                    //PX4_INFO("Desired yaw angle  : \t%8.8f", (double)take_outputs_pp.desired_yaw_pp);

                    //*********** gokhan patch ********************

                    float error_yaw = take_outputs_pp.desired_yaw_pp - local_pos.yaw;
                    if(error_yaw > (float)M_PI)
                    {
                        take_outputs_pp.desired_yaw_pp  = take_outputs_pp.desired_yaw_pp  - (float)(M_PI) * 2.0f;
                    }
                    else if(error_yaw < (float)(-M_PI) )
                    {
                        take_outputs_pp.desired_yaw_pp  = take_outputs_pp.desired_yaw_pp  + (float)M_PI * 2.0f;
                    }
                    else
                    {
                        // do nothing
                    }

                    //*********** gokhan patch ********************


                    mpc_return = mpc_controller_3dof(&local_pos, &att,
                                    U_L, U_R, prev_state_vector,
                                    take_outputs_pp.desired_surge_pp,
                                    take_outputs_pp.desired_yaw_pp);

                    prev_state_vector = mpc_return.return_state_vector;

                    //PX4_INFO("Previous State Vector");
                    //prev_state_vector.print();
                    //prev_state_vector.setZero();

                    //PX4_INFO("mpc_return.delta_left_thrust: \t%8.8f", (double)mpc_return.delta_left_thrust);
                    //PX4_INFO("mpc_return.delta_right_thrust: \t%8.8f", (double)mpc_return.delta_right_thrust);

                    U_L = U_L + mpc_return.delta_left_thrust;
                    U_R = U_R + mpc_return.delta_right_thrust;

                    way_point_index = take_outputs_pp.wp_index_pp;
                    yaw_state = take_outputs_pp.yaw_state;
                    accumulation = take_outputs_pp.accumulation_pp;
                    psi_last = take_outputs_pp.psi_last_pp;
                    accumulation = take_outputs_pp.accumulation_pp;

                    //PX4_INFO("Delta_L_th, L_th: \t%8.4f \t%8.4f", (double)mpc_return.delta_left_thrust, (double)U_L);
                    //PX4_INFO("Delta_R_th, R_th: \t%8.4f \t%8.4f", (double)mpc_return.delta_right_thrust,(double)U_R);

                    if (U_L > U_MAX){
                        U_L = U_MAX;
                    }
                    else if (U_L < U_MIN){
                        U_L = U_MIN;
                    }

                    if (U_R > U_MAX){
                        U_R = U_MAX;
                    }
                    else if (U_R < U_MIN){
                        U_R = U_MIN;
                    }

                    float U_L_Out = U_L/40.0f;

                    if ( U_L_Out > MIXER_LIM){
                        actuators.control[0] = MIXER_LIM;
                    }
                    else if (U_L_Out< -MIXER_LIM){
                        actuators.control[0] = -MIXER_LIM;
                    }
                    else
                    {
                        actuators.control[0] = U_L_Out;
                    }

                    float U_R_Out = U_R/40.0f;
                    if (U_R_Out > MIXER_LIM){
                        actuators.control[1] = MIXER_LIM;
                    }
                    else if (U_R_Out < -MIXER_LIM){
                        actuators.control[1] = -MIXER_LIM;
                    }
                    else
                    {
                        actuators.control[1] = U_R_Out;
                    }

                    // Log the state variables to vehicle_reference_states.msg
                    reference_states.reference_surge_speed = take_outputs_pp.desired_surge_pp;
                    reference_states.reference_yaw_angle = take_outputs_pp.desired_yaw_pp;
                    reference_states.finished = take_outputs_pp.finished_pp;
                    reference_states.number_of_way_point = number_of_wp;
                    reference_states.way_point_index = way_point_index;

                    if (take_outputs_pp.finished_pp == true)
                    {
                        thread_should_exit = true;
                    }

                }
                    else
                    {
                        PX4_INFO("Not updated!");
                    }

				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {
                    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

					if (verbose) {
						warnx("published");
					}
				}
            }

            reference_states.x = local_pos.x;
            reference_states.y = local_pos.y;
            reference_states.yaw = local_pos.yaw;
            surge_speed = (local_pos.vx)*(float)cos(local_pos.yaw) + (local_pos.vy)*(float)sin(local_pos.yaw);
            reference_states.vx = surge_speed;
            reference_states.vy = -(local_pos.vx)*(float)sin(local_pos.yaw) + (local_pos.vy)*(float)cos(local_pos.yaw);
            reference_states.yawspeed = att.yawspeed;

            reference_states.left_thruster_force = U_L;
            reference_states.right_thruster_force = U_R;

            output_error_cost = (take_outputs_pp.desired_surge_pp - surge_speed)*
                                surge_weight * (take_outputs_pp.desired_surge_pp - surge_speed)+
                                (take_outputs_pp.desired_yaw_pp - att.yawspeed)*
                                yaw_weight * (take_outputs_pp.desired_yaw_pp - att.yawspeed);


            control_effort_cost = mpc_return.delta_left_thrust * input_weight * mpc_return.delta_left_thrust +
                                  mpc_return.delta_right_thrust * input_weight * mpc_return.delta_right_thrust;

            total_cost = output_error_cost + control_effort_cost;

            reference_states.output_error_cost = output_error_cost;
            reference_states.control_effort_cost = control_effort_cost;
            reference_states.total_cost = total_cost;
            //PX4_INFO("output_error_cost  : \t%8.8f", (double)output_error_cost);
            //PX4_INFO("control_effort_cost: \t%8.8f", (double)control_effort_cost);
            //PX4_INFO("total_cost         : \t%8.8f", (double)total_cost);


            if (way_point_change != way_point_index)
            {
                way_point_change = way_point_index;
                PX4_WARN("WAY_POINT: %d", way_point_index);
            }


            time_loop_exec = hrt_elapsed_time(&time_loop_start);
            //PX4_INFO("Execution Time of Loop (in microsecond): \t%d", (int)time_loop_exec);

            reference_states.loop_time = time_loop_exec;

            orb_publish(ORB_ID(vehicle_reference_states),reference_states_pub, &reference_states);
        }
	}

	warnx("exiting, stopping all motors.");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
		actuators.control[i] = 0.0f;
	}

    actuators.timestamp = hrt_absolute_time();
    reference_states.timestamp = hrt_absolute_time();

    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
    orb_publish(ORB_ID(vehicle_reference_states),reference_states_pub, &reference_states);

	fflush(stdout);

	return 0;
}

struct filtered_references lowpass_filter_for_references(float prev_filtered_yaw_ref,
                                                         float desired_yaw,
                                                         float yaw_filter_coeffient,
                                                         float prev_filtered_surge_ref,
                                                         float desired_surge,
                                                         float surge_filter_coeffient)
{
    struct filtered_references filter_outputs;

    //PX4_INFO("yaw_filter_coeffient: \t%8.8f", (double)yaw_filter_coeffient);
    //PX4_INFO("surge_filter_coeffient: \t%8.8f", (double)surge_filter_coeffient);

    filter_outputs.filtered_yaw = (yaw_filter_coeffient)*prev_filtered_yaw_ref +
                                  (1-yaw_filter_coeffient) * desired_yaw;

    filter_outputs.filtered_surge = (surge_filter_coeffient)*prev_filtered_surge_ref +
                                    (1-surge_filter_coeffient) * desired_surge;

    return filter_outputs;
}


/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: usv_mpc_control {start|stop|status}\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int usv_mpc_control(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
        return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("running");
			/* this is not an error */
            //exit(0);
            return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("usv_mpc_control",
						 SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 0,
                         30000,
						 usv_mpc_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
        //exit(0);
        return 0;
	}

	if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        //exit(0);
        return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");

		} else {
			warnx("not started");
		}

        //exit(0);
        return 0;
	}

	usage("unrecognized command");
    //exit(1);
    return 1;
}
