/*
 * op3_walking_parameter.h
 *
 *  Created on: 2016. 5. 16.
 *      Author: JungKM
 */

#ifndef OP3_WALKING_PARAMETER_H_
#define OP3_WALKING_PARAMETER_H_

class WalkingTimeParameter
{
private:
    double periodtime;
    double dsp_ratio;
    double ssp_ratio;
    double x_swap_periodtime;
    double x_move_periodtime;
    double y_swap_periodtime;
    double y_move_periodtime;
    double z_swap_periodtime;
    double z_move_periodtime;
    double a_move_periodtime;
    double ssp_time;
    double ssp_time_start_l;
    double ssp_time_end_l;
    double ssp_time_start_r;
    double ssp_time_end_r;
    double phase1_time;
    double phase2_time;
    double phase3_time;

public:
    enum
    {
        PHASE0 = 0,
        PHASE1 = 1,
        PHASE2 = 2,
        PHASE3 = 3
    };
};

class WalkingMovementParameter
{
private:

};

class WalkingBalanceParameter
{

};



#endif /* OP3_WALKING_PARAMETER_H_ */
