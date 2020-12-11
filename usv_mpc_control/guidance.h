#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <math.h>
#include <uORB/topics/vehicle_local_position.h>


struct pp_guidance_outpus
{
    int   wp_index_pp;      //way point index
    bool  finished_pp;      // check last way point
    float desired_surge_pp; // desired surge from guidance
    float desired_yaw_pp;   // desired yaw from guidance
    float psi_last_pp;      // last yaw angle value
    float accumulation_pp;  // accumulation value for guidance algorithm
    int   yaw_state;        // yaw state in the coordinate plane
};

/**
 * Guidance Algorithm for Controller
 *
 * Pure Pursuit Guidance produces reference surge speed and yaw angle for controller
 * other states are not controlled
 *
 */
struct pp_guidance_outpus pp_guidance(struct vehicle_local_position_s *local_pos,
                                      int way_point_index, int number_of_wp,
                                      int state, float psi_last, float accumulation,
                                      float way_point_info[9][5]);
/*
 * signum function
 * value is true if value >= 0
 * else
 * value is false
 */
bool sgn(double value);


#endif // GUIDANCE_H
