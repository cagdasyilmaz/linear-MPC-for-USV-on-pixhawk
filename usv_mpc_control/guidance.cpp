#include "guidance.h"


bool sgn(double value)
{
    if (value < 0) return false;
    if (value >= 0) return true;
    return false;
}

struct pp_guidance_outpus pp_guidance(struct vehicle_local_position_s *local_pos,
                                      int way_point_index, int number_of_wp,
                                      int state, float psi_last, float accumulation,
                                      float way_point_info[9][5])
{
    struct pp_guidance_outpus outputs;
    float p_tilda_x = 0.0f; // x component of guidance vector
    float p_tilda_y = 0.0f; // y component of guidance vector
    float p_euclid = 0.0f;  // euclidian length of guidance vector
    float psi_now = 0.0f;   // yaw angle from atan2 function
    float accumulate = 0.0f;
    int n = 0;
    float reminder_pp = 0.0f;
    float psi_mapped = 0.0f;
    int previous_way_point_index = 0;
    previous_way_point_index = way_point_index - 1;


    outputs.finished_pp = false;
    outputs.desired_surge_pp = 0.0f;
    outputs.desired_yaw_pp = 0.0f;

    /* Determine whether motion is forward or backward
     * Forward  : 1
     * Backward : 2
     */
    int forward_or_backward = (int)way_point_info[way_point_index][2];

    //PX4_INFO("Way Point Index: \t%d", (int)way_point_index);
    //PX4_INFO("Forward or Backward: \t%d", (int)forward_or_backward);
    //PX4_INFO("Forward or Backward: \t%8.4f", (double)way_point_info[way_point_index][2]);

    if(forward_or_backward == 2){
        // Backward guidance vector is handled.
        //PX4_INFO("Motion is backward: \t%d", (int)forward_or_backward);
        //p_tilda_x = local_pos->x - pp.wp_x_1;
        p_tilda_x = local_pos->x - way_point_info[way_point_index][0];
        //p_tilda_y = local_pos->y - pp.wp_y_1;
        p_tilda_y = local_pos->y - way_point_info[way_point_index][1];
    }
    else if (forward_or_backward == 1)
    {
        // Forward guidance vector is handled.
        //PX4_INFO("Motion is forward: \t%d", (int)forward_or_backward);
        p_tilda_x = way_point_info[way_point_index][0] - local_pos->x;
        p_tilda_y = way_point_info[way_point_index][1] - local_pos->y;
    }

    p_euclid = sqrt(p_tilda_x*p_tilda_x + p_tilda_y*p_tilda_y);
    //PX4_INFO("Euclid length of guidance vector: /t%8.4f", (double)p_euclid);

    // (p_euclid < Way_Point_COA)
    if (p_euclid < way_point_info[way_point_index][3])
    {
        way_point_index = way_point_index + 1;

        if (way_point_index == number_of_wp)
        {
            outputs.finished_pp = true;
        }
        else
        {
            /* Make tilda calculation for next waypoint
             * now it is making calculation on older way point
             */
            if(forward_or_backward == 2){
                // Backward guidance vector is handled.
                p_tilda_x = local_pos->x - way_point_info[way_point_index][0];
                p_tilda_y = local_pos->y - way_point_info[way_point_index][1];
            }
            else if (forward_or_backward == 1)
            {
                // Forward guidance vector is handled.
                p_tilda_x = way_point_info[way_point_index][0] - local_pos->x;
                p_tilda_y = way_point_info[way_point_index][1] - local_pos->y;
            }
            //p_euclid = sqrt(p_tilda_x*p_tilda_x + p_tilda_y*p_tilda_y);
        }
    }

    if (previous_way_point_index == -1)
    {
        outputs.desired_surge_pp = way_point_info[way_point_index][4];
    }
    else
    {
        p_tilda_x = way_point_info[previous_way_point_index][0] - local_pos->x;
        p_tilda_y = way_point_info[previous_way_point_index][1] - local_pos->y;
        p_euclid = sqrt(p_tilda_x*p_tilda_x + p_tilda_y*p_tilda_y);

        if (p_euclid < way_point_info[previous_way_point_index][3])
        {
            outputs.desired_surge_pp = way_point_info[previous_way_point_index][4]/2;
        }
        else
        {
            if(outputs.finished_pp == true)
            {
                outputs.desired_surge_pp = way_point_info[way_point_index-1][4];
            }
            else
            {
                outputs.desired_surge_pp = way_point_info[way_point_index][4];
                //PX4_INFO("Surge Speed: \t%8.4f", (double)way_point_info[way_point_index][4]);
            }
        }
    }

    /*
     * Calcaulate desired yaw angle
     */
    if(sgn(outputs.desired_surge_pp) == false) // backward motion
    {
        if(outputs.wp_index_pp == number_of_wp) // last way point
        {
            p_tilda_x = local_pos->x - way_point_info[number_of_wp-1][0];
            p_tilda_y = local_pos->y - way_point_info[number_of_wp-1][1];
            psi_now = atan2(p_tilda_y, p_tilda_x);
        }
        else
        {
            p_tilda_x = local_pos->x - way_point_info[way_point_index][0];
            p_tilda_y = local_pos->y - way_point_info[way_point_index][1];
            psi_now = atan2(p_tilda_y, p_tilda_x);
        }

    }
    else // forward motion
    {
        if(outputs.wp_index_pp == number_of_wp) // last way point
        {
            p_tilda_x = way_point_info[number_of_wp-1][0] - local_pos->x;
            p_tilda_y = way_point_info[number_of_wp-1][1] - local_pos->y;
            psi_now = atan2(p_tilda_y, p_tilda_x);
        }
        else
        {
            p_tilda_x = way_point_info[way_point_index][0] - local_pos->x;
            p_tilda_y = way_point_info[way_point_index][1] - local_pos->y;
            psi_now = atan2(p_tilda_y, p_tilda_x);
        }

    }

    /* Check psi_now whether has a valid number or not*/
    if (isnan(psi_now))
    {
        psi_now = 0.0f;
    }

    /* The mapping algorithm is implemented as */
    if (sgn(p_tilda_y) == true && sgn(p_tilda_x) == true)
    {
        // Now in the 1. quadrant
        if (state == 3)
        {
            if( (psi_now + (float)fabs((double)psi_last)) <= (float)M_PI)
            {
                accumulate = psi_now - psi_last;
            }
            else
            {
                accumulate = psi_last - psi_now + (float)(2*M_PI);
            }
        }
        else //state == 1, 2 or 4
        {
            accumulate = psi_now - psi_last;
        }
        state = 1;
    }
    else if (sgn(p_tilda_y) == false && sgn(p_tilda_x) == true)
    {
        // Now in the 2. quadrant
        if (state == 4)
        {
            if ( ((float)fabs(double(psi_now)) + psi_last) <= float(M_PI))
            {
                accumulate = psi_now - psi_last;
            }
            else
            {
                accumulate = psi_now - psi_last + (float)(2*M_PI);
            }
        }
        else //state == 1, 2 or 3
        {
            accumulate = psi_now - psi_last;
        }
        state = 2;
    }
    else if (sgn(p_tilda_y) == false && sgn(p_tilda_x) == false)
    {
        // Now in the 3. quadrant
        if (state == 1)
        {
            if (((float)fabs((double)psi_now) + psi_last) <= float(M_PI))
            {
                accumulate = psi_now - psi_last;
            }
            else
            {
                accumulate = psi_now - psi_last + (float)(2*M_PI);
            }
        }
        else if (state == 4)
        {
            accumulate = psi_now - psi_last + (float)(2*M_PI);
        }
        else //state == 2 or 3
        {
            accumulate = psi_now - psi_last;
        }
        state = 3;
    }
    else
    {
        // (sgn(p_tilda_y) == true && sgn(p_tilda_x) == false)
        // Now in the 4. quadrant
        if (state == 2)
        {
            if (psi_now + (float)fabs((double)psi_last) <= float(M_PI))
            {
                accumulate = psi_now - psi_last;
            }
            else
            {
                accumulate = psi_last - psi_now + (float)(2*M_PI);
            }
        }
        else if (state == 3)
        {
            accumulate = psi_now - psi_last - (float)(2*M_PI);
        }
        else //state == 1 or 4
        {
            accumulate = psi_now - psi_last;
        }
        state = 4;
    }

    outputs.accumulation_pp = accumulation + accumulate;
    psi_last = psi_now;
    outputs.psi_last_pp = psi_last;

    if(sgn(outputs.accumulation_pp) == true) //sign(psi) == 1
    {
        n = (int)floor((double)(outputs.accumulation_pp/((float)(M_PI))));
    }
    else if(sgn(outputs.accumulation_pp) == false) //sign(psi) == -1
    {
        n = (int)ceil((double)(outputs.accumulation_pp/((float)(M_PI))));
    }
    else
    {
        n = 0;
    }

    reminder_pp = n % 2;

    if( (int)reminder_pp == 0)
    {
        psi_mapped = accumulation - n*float(M_PI);
    }
    else
    {
        if (sgn(reminder_pp) == true) //sign(remiender)== 1
        {
            psi_mapped = outputs.accumulation_pp - (n+1)*((float)(M_PI));
        }
        else // (sgn(reminder_pp) == false) //sign(remiender)== -1
        {
            psi_mapped = outputs.accumulation_pp - (n-1)*((float)(M_PI));
        }
    }
    outputs.wp_index_pp = way_point_index;
    outputs.desired_yaw_pp = psi_mapped;
    outputs.yaw_state = state;
    return outputs;
}
