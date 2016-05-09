/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */


#include "op2_kinematics_dynamics/OP2KinematicsDynamics.h"
#include <iostream>

namespace ROBOTIS
{

OP2KinematicsDynamics::OP2KinematicsDynamics() {}
OP2KinematicsDynamics::~OP2KinematicsDynamics() {}

OP2KinematicsDynamics::OP2KinematicsDynamics(TREE_SELECT tree)
{
    for ( int id = 0; id <= ALL_JOINT_ID; id++ )
        op2_link_data[ id ] = new LinkData();

    // Todo : Make tree for OP2
    if ( tree == WHOLE_BODY )
    {
        op2_link_data[0]->name                  =   "base";
        op2_link_data[0]->parent                =   -1;
        op2_link_data[0]->sibling               =   -1;
        op2_link_data[0]->child                 =   38;
        op2_link_data[0]->mass                  =   0.0;
        op2_link_data[0]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[0]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[0]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[0]->joint_limit_max       =   100.0;
        op2_link_data[0]->joint_limit_min       =   -100.0;
        op2_link_data[0]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- passive joint -----*/

        op2_link_data[38]->name                  =   "passive_x";
        op2_link_data[38]->parent                =   0;
        op2_link_data[38]->sibling               =   -1;
        op2_link_data[38]->child                 =   39;
        op2_link_data[38]->mass                  =   0.0;
        op2_link_data[38]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[38]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[38]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[38]->joint_limit_max       =   100.0;
        op2_link_data[38]->joint_limit_min       =   -100.0;
        op2_link_data[38]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        op2_link_data[39]->name                  =   "passive_y";
        op2_link_data[39]->parent                =   38;
        op2_link_data[39]->sibling               =   -1;
        op2_link_data[39]->child                 =   40;
        op2_link_data[39]->mass                  =   0.0;
        op2_link_data[39]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[39]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[39]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[39]->joint_limit_max       =   100.0;
        op2_link_data[39]->joint_limit_min       =   -100.0;
        op2_link_data[39]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        op2_link_data[40]->name                  =   "passive_z";
        op2_link_data[40]->parent                =   39;
        op2_link_data[40]->sibling               =   -1;
        op2_link_data[40]->child                 =   41;
        op2_link_data[40]->mass                  =   0.0;
        op2_link_data[40]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.801 );
        op2_link_data[40]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[40]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[40]->joint_limit_max       =   100.0;
        op2_link_data[40]->joint_limit_min       =   -100.0;
        op2_link_data[40]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        op2_link_data[41]->name                  =   "passive_roll";
        op2_link_data[41]->parent                =   40;
        op2_link_data[41]->sibling               =   -1;
        op2_link_data[41]->child                 =   42;
        op2_link_data[41]->mass                  =   0.0;
        op2_link_data[41]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[41]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        op2_link_data[41]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[41]->joint_limit_max       =   100.0;
        op2_link_data[41]->joint_limit_min       =   -100.0;
        op2_link_data[41]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        op2_link_data[42]->name                  =   "passive_pitch";
        op2_link_data[42]->parent                =   41;
        op2_link_data[42]->sibling               =   -1;
        op2_link_data[42]->child                 =   43;
        op2_link_data[42]->mass                  =   0.0;
        op2_link_data[42]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[42]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[42]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[42]->joint_limit_max       =   100.0;
        op2_link_data[42]->joint_limit_min       =   -100.0;
        op2_link_data[42]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        op2_link_data[43]->name                  =   "passive_yaw";
        op2_link_data[43]->parent                =   42;
        op2_link_data[43]->sibling               =   -1;
        op2_link_data[43]->child                 =   44;
        op2_link_data[43]->mass                  =   0.0;
        op2_link_data[43]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[43]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[43]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[43]->joint_limit_max       =   100.0;
        op2_link_data[43]->joint_limit_min       =   -100.0;
        op2_link_data[43]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- body -----*/

        // pelvis_link
        op2_link_data[44]->name                  =   "pelvis";
        op2_link_data[44]->parent                =   43;
        op2_link_data[44]->sibling               =   -1;
        op2_link_data[44]->child                 =   27;
        op2_link_data[44]->mass                  =   6.869;
        op2_link_data[44]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[44]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[44]->center_of_mass        =   transitionXYZ( -0.011 , 0.000 , 0.058 );
        op2_link_data[44]->joint_limit_max       =   100.0;
        op2_link_data[44]->joint_limit_min       =   -100.0;
        op2_link_data[44]->inertia               =   inertiaXYZ( 0.03603 , 0.00000 , 0.00016 , 0.02210 , 0.00000 , 0.03830 );

        // chest_link
        op2_link_data[27]->name                  =   "torso_y";
        op2_link_data[27]->parent                =   44;
        op2_link_data[27]->sibling               =   15;
        op2_link_data[27]->child                 =   28;
        op2_link_data[27]->mass                  =   5.383;
        op2_link_data[27]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.171 );
        op2_link_data[27]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[27]->center_of_mass        =   transitionXYZ( -0.007 , 0.000 , 0.109 );
        op2_link_data[27]->joint_limit_max       =   0.6 * M_PI;
        op2_link_data[27]->joint_limit_min       =   -0.6 * M_PI;
        op2_link_data[27]->inertia               =   inertiaXYZ( 0.04710 , 0.00000 , 0.00036 , 0.02554 , 0.00000 , 0.03094 );

        /* ----- head -----*/

        // head_yaw
        op2_link_data[28]->name                  =   "head_y";
        op2_link_data[28]->parent                =   27;
        op2_link_data[28]->sibling               =   1;
        op2_link_data[28]->child                 =   29;
        op2_link_data[28]->mass                  =   0.087;
        op2_link_data[28]->relative_position     =   transitionXYZ( 0.0 , 0.0 , 0.229 );
        op2_link_data[28]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[28]->center_of_mass        =   transitionXYZ( 0.000 , -0.002 , 0.010 );
        op2_link_data[28]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[28]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[28]->inertia               =   inertiaXYZ( 0.00011 , 0.00000 , 0.00000 , 0.00003 , 0.00000 , 0.00012 );

        // head_pitch
        op2_link_data[29]->name                  =   "head_p";
        op2_link_data[29]->parent                =   28;
        op2_link_data[29]->sibling               =   -1;
        op2_link_data[29]->child                 =   -1;
        op2_link_data[29]->mass                  =   0.724;
        op2_link_data[29]->relative_position     =   transitionXYZ( 0.0 , -0.04500 , 0.03900 );
        op2_link_data[29]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[29]->center_of_mass        =   transitionXYZ( 0.009 , 0.046 , 0.022 );
        op2_link_data[29]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[29]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[29]->inertia               =   inertiaXYZ( 0.00113 , 0.00001 , -0.00005 , 0.00114 , 0.00002 , 0.00084 );

        /*----- right arm -----*/

        // right arm shoulder pitch 1
        op2_link_data[1]->name                  =   "r_arm_sh_p1";
        op2_link_data[1]->parent                =   27;
        op2_link_data[1]->sibling               =   2;
        op2_link_data[1]->child                 =   3;
        op2_link_data[1]->mass                  =   0.194;
        op2_link_data[1]->relative_position     =   transitionXYZ( 0.000 , -0.152 , 0.160 );
        op2_link_data[1]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        op2_link_data[1]->center_of_mass        =   transitionXYZ( -0.003 , -0.020 , -0.005 );
        op2_link_data[1]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[1]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[1]->inertia               =   inertiaXYZ( 0.00018 , 0.0 , 0.0 , 0.00058 , -0.00004 , 0.00057 );

        // right arm shoulder roll
        op2_link_data[3]->name                  =   "r_arm_sh_r";
        op2_link_data[3]->parent                =   1;
        op2_link_data[3]->sibling               =   -1;
        op2_link_data[3]->child                 =   5;
        op2_link_data[3]->mass                  =   0.875;
        op2_link_data[3]->relative_position     =   transitionXYZ( 0.057 , -0.060 , -0.039 );
        op2_link_data[3]->joint_axis            =   transitionXYZ( -1.0 , 0.0 , 0.0 );
        op2_link_data[3]->center_of_mass        =   transitionXYZ( -0.060 , -0.002 , 0.000 );
        op2_link_data[3]->joint_limit_max       =   0.3 * M_PI;
        op2_link_data[3]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[3]->inertia               =   inertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

        // right arm shoulder pitch 2
        op2_link_data[5]->name                  =   "r_arm_sh_p2";
        op2_link_data[5]->parent                =   3;
        op2_link_data[5]->sibling               =   -1;
        op2_link_data[5]->child                 =   7;
        op2_link_data[5]->mass                  =   1.122;
        op2_link_data[5]->relative_position     =   transitionXYZ( -0.057 , -0.033 , 0.000 );
        op2_link_data[5]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        op2_link_data[5]->center_of_mass        =   transitionXYZ( 0.000 , -0.073 , 0.000 );
        op2_link_data[5]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[5]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[5]->inertia               =   inertiaXYZ( 0.00277 , 0.00002 , -0.00001 , 0.00090 , 0.00004 , 0.00255 );

        // right arm elbow yaw
        op2_link_data[7]->name                  =   "r_arm_el_y";
        op2_link_data[7]->parent                =   5;
        op2_link_data[7]->sibling               =   -1;
        op2_link_data[7]->child                 =   9;
        op2_link_data[7]->mass                  =   1.357;
        op2_link_data[7]->relative_position     =   transitionXYZ( 0.03000 , -0.18700 , 0.05700 );
        op2_link_data[7]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        op2_link_data[7]->center_of_mass        =   transitionXYZ( 0.042 , -0.012 , -0.058 );
        op2_link_data[7]->joint_limit_max       =   0.4 * M_PI;
        op2_link_data[7]->joint_limit_min       =   -0.4 * M_PI;
        op2_link_data[7]->inertia               =   inertiaXYZ( 0.00152 , 0.00100 , -0.00006 , 0.00560 , 0.00002 , 0.00528 );

        // right arm wrist roll
        op2_link_data[9]->name                  =   "r_arm_wr_r";
        op2_link_data[9]->parent                =   7;
        op2_link_data[9]->sibling               =   -1;
        op2_link_data[9]->child                 =   11;
        op2_link_data[9]->mass                  =   0.087;
        op2_link_data[9]->relative_position     =   transitionXYZ( 0.171 , -0.030 , -0.057 );
        op2_link_data[9]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        op2_link_data[9]->center_of_mass        =   transitionXYZ( 0.010 , 0.000 , -0.002 );
        op2_link_data[9]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[9]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[9]->inertia               =   inertiaXYZ( 0.00012 , 0.00000 , 0.00000 , 0.00011 , 0.00000 , 0.00003 );

        // right arm wrist yaw
        op2_link_data[11]->name                  =   "r_arm_wr_y";
        op2_link_data[11]->parent                =   9;
        op2_link_data[11]->sibling               =   -1;
        op2_link_data[11]->child                 =   13;
        op2_link_data[11]->mass                  =   0.768;
        op2_link_data[11]->relative_position     =   transitionXYZ( 0.039 , 0.000 , 0.045 );
        op2_link_data[11]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        op2_link_data[11]->center_of_mass        =   transitionXYZ( 0.023 , -0.001 , -0.046 );
        op2_link_data[11]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[11]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[11]->inertia               =   inertiaXYZ( 0.00059 , 0.00002 , -0.00002 , 0.00078 , 0.00000 , 0.00078 );

        // right arm wrist pitch
        op2_link_data[13]->name                  =   "r_arm_wr_p";
        op2_link_data[13]->parent                =   11;
        op2_link_data[13]->sibling               =   -1;
        op2_link_data[13]->child                 =   31;
        op2_link_data[13]->mass                  =   0.565;
        op2_link_data[13]->relative_position     =   transitionXYZ( 0.045 , 0.045 , -0.045 );
        op2_link_data[13]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[13]->center_of_mass        =   transitionXYZ( 0.065 , -0.045 , 0.000 );
        op2_link_data[13]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[13]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[13]->inertia               =   inertiaXYZ( 0.00047 , 0.00001 , 0.00000 , 0.00042 , 0.00000 , 0.00058 );

        // right arm gripper
        op2_link_data[31]->name                  =   "r_arm_grip";
        op2_link_data[31]->parent                =   13;
        op2_link_data[31]->sibling               =   33;
        op2_link_data[31]->child                 =   -1;
        op2_link_data[31]->mass                  =   0.013;
        op2_link_data[31]->relative_position     =   transitionXYZ( 0.088 , -0.058 , 0.000 );
        op2_link_data[31]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[31]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[31]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[31]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[31]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // right arm gripper 1
        op2_link_data[33]->name                  =   "r_arm_grip_1";
        op2_link_data[33]->parent                =   13;
        op2_link_data[33]->sibling               =   35;
        op2_link_data[33]->child                 =   -1;
        op2_link_data[33]->mass                  =   0.013;
        op2_link_data[33]->relative_position     =   transitionXYZ( 0.088 , -0.032 , 0.000 );
        op2_link_data[33]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[33]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[33]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[33]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[33]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // right arm end effector
        op2_link_data[35]->name                  =   "r_arm_end";
        op2_link_data[35]->parent                =   13;
        op2_link_data[35]->sibling               =   -1;
        op2_link_data[35]->child                 =   -1;
        op2_link_data[35]->mass                  =   0.0;
        op2_link_data[35]->relative_position     =   transitionXYZ( 0.145 , -0.045 , 0.0 );
        op2_link_data[35]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[35]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[35]->joint_limit_max       =   100.0;
        op2_link_data[35]->joint_limit_min       =   -100.0;
        op2_link_data[35]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /*----- left arm -----*/

        // left arm shoulder pitch 1
        op2_link_data[2]->name                  =   "l_arm_sh_p1";
        op2_link_data[2]->parent                =   27;
        op2_link_data[2]->sibling               =   -1;
        op2_link_data[2]->child                 =   4;
        op2_link_data[2]->mass                  =   0.194;
        op2_link_data[2]->relative_position     =   transitionXYZ( 0.000 , 0.152 , 0.160 );
        op2_link_data[2]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[2]->center_of_mass        =   transitionXYZ( -0.003 , 0.020 , -0.005 );
        op2_link_data[2]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[2]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[2]->inertia               =   inertiaXYZ( 0.00018 , 0.00000 , 0.00000 , 0.00058 , 0.00004 , 0.00057 );

        // left arm shoulder roll
        op2_link_data[4]->name                  =   "l_arm_sh_r";
        op2_link_data[4]->parent                =   2;
        op2_link_data[4]->sibling               =   -1;
        op2_link_data[4]->child                 =   6;
        op2_link_data[4]->mass                  =   0.875;
        op2_link_data[4]->relative_position     =   transitionXYZ( 0.057 , 0.060 , -0.039 );
        op2_link_data[4]->joint_axis            =   transitionXYZ( -1.0 , 0.0 , 0.0 );
        op2_link_data[4]->center_of_mass        =   transitionXYZ( -0.060 , 0.002 , 0.000 );
        op2_link_data[4]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[4]->joint_limit_min       =   -0.3 * M_PI;
        op2_link_data[4]->inertia               =   inertiaXYZ( 0.00043 , 0.00000 , 0.00000 , 0.00112 , 0.00000 , 0.00113 );

        // left arm shoulder pitch 2
        op2_link_data[6]->name                  =   "l_arm_sh_p2";
        op2_link_data[6]->parent                =   4;
        op2_link_data[6]->sibling               =   -1;
        op2_link_data[6]->child                 =   8;
        op2_link_data[6]->mass                  =   1.122;
        op2_link_data[6]->relative_position     =   transitionXYZ( -0.057 , 0.033 , 0.000 );
        op2_link_data[6]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[6]->center_of_mass        =   transitionXYZ( 0.000 , 0.073 , 0.000 );
        op2_link_data[6]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[6]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[6]->inertia               =   inertiaXYZ( 0.00277 , -0.00002 , -0.00001 , 0.00090 , -0.00004 , 0.00255 );

        // left arm elbow yaw
        op2_link_data[8]->name                  =   "l_arm_el_y";
        op2_link_data[8]->parent                =   6;
        op2_link_data[8]->sibling               =   -1;
        op2_link_data[8]->child                 =   10;
        op2_link_data[8]->mass                  =   1.357;
        op2_link_data[8]->relative_position     =   transitionXYZ( 0.030 , 0.187 , 0.057 );
        op2_link_data[8]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        op2_link_data[8]->center_of_mass        =   transitionXYZ( 0.042 , 0.012 , -0.058 );
        op2_link_data[8]->joint_limit_max       =   0.4 * M_PI;
        op2_link_data[8]->joint_limit_min       =   -0.4 * M_PI;
        op2_link_data[8]->inertia               =   inertiaXYZ( 0.00152 , -0.00100 , -0.00006 , 0.00560 , -0.00002 , 0.00528 );

        // left arm wrist roll
        op2_link_data[10]->name                  =   "l_arm_wr_r";
        op2_link_data[10]->parent                =   8;
        op2_link_data[10]->sibling               =   -1;
        op2_link_data[10]->child                 =   12;
        op2_link_data[10]->mass                  =   0.087;
        op2_link_data[10]->relative_position     =   transitionXYZ( 0.171 , 0.030 , -0.057 );
        op2_link_data[10]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        op2_link_data[10]->center_of_mass        =   transitionXYZ( 0.010 , 0.000 , 0.002 );
        op2_link_data[10]->joint_limit_max       =   0.5 * M_PI;
        op2_link_data[10]->joint_limit_min       =   -0.5 * M_PI;
        op2_link_data[10]->inertia               =   inertiaXYZ( 0.00012 , 0.00000 , 0.00000 , 0.00011 , 0.00000 , 0.00003 );

        // left arm wrist yaw
        op2_link_data[12]->name                  =   "l_arm_wr_y";
        op2_link_data[12]->parent                =   10;
        op2_link_data[12]->sibling               =   -1;
        op2_link_data[12]->child                 =   14;
        op2_link_data[12]->mass                  =   0.768;
        op2_link_data[12]->relative_position     =   transitionXYZ( 0.039 , 0.000 , 0.045 );
        op2_link_data[12]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        op2_link_data[12]->center_of_mass        =   transitionXYZ( 0.023 , 0.001 , -0.046 );
        op2_link_data[12]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[12]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[12]->inertia               =   inertiaXYZ( 0.00059 , -0.00002 , -0.00002 , 0.00078 , 0.00000 , 0.00078 );

        // left arm wrist pitch
        op2_link_data[14]->name                  =   "l_arm_wr_p";
        op2_link_data[14]->parent                =   12;
        op2_link_data[14]->sibling               =   -1;
        op2_link_data[14]->child                 =   30;
        op2_link_data[14]->mass                  =   0.08709;
        op2_link_data[14]->relative_position     =   transitionXYZ( 0.045 , -0.045 , -0.045 );
        op2_link_data[14]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        op2_link_data[14]->center_of_mass        =   transitionXYZ( 0.065 , 0.045 , 0.000 );
        op2_link_data[14]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[14]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[14]->inertia               =   inertiaXYZ( 0.00047 , -0.00001 , 0.00000 , 0.00042 , 0.00000 , 0.00058 );

        // left arm gripper
        op2_link_data[30]->name                  =   "l_arm_grip";
        op2_link_data[30]->parent                =   14;
        op2_link_data[30]->sibling               =   32;
        op2_link_data[30]->child                 =   -1;
        op2_link_data[30]->mass                  =   0.013;
        op2_link_data[30]->relative_position     =   transitionXYZ( 0.088 , 0.058 , 0.000 );
        op2_link_data[30]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[30]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[30]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[30]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[30]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // left arm gripper_1
        op2_link_data[32]->name                  =   "l_arm_grip_1";
        op2_link_data[32]->parent                =   14;
        op2_link_data[32]->sibling               =   34;
        op2_link_data[32]->child                 =   -1;
        op2_link_data[32]->mass                  =   0.013;
        op2_link_data[32]->relative_position     =   transitionXYZ( 0.088 , 0.032 , 0.000 );
        op2_link_data[32]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 1.0 );
        op2_link_data[32]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[32]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[32]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[32]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        // left arm end effector
        op2_link_data[34]->name                  =   "l_arm_end";
        op2_link_data[34]->parent                =   14;
        op2_link_data[34]->sibling               =   -1;
        op2_link_data[34]->child                 =   -1;
        op2_link_data[34]->mass                  =   0.0;
        op2_link_data[34]->relative_position     =   transitionXYZ( 0.145 , 0.045 , 0.0 );
        op2_link_data[34]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[34]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[34]->joint_limit_max       =   100.0;
        op2_link_data[34]->joint_limit_min       =   -100.0;
        op2_link_data[34]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- right leg -----*/

        // right leg hip yaw
        op2_link_data[15]->name                  =   "r_leg_hip_y";
        op2_link_data[15]->parent                =   44;
        op2_link_data[15]->sibling               =   16;
        op2_link_data[15]->child                 =   17;
        op2_link_data[15]->mass                  =   0.243;
        op2_link_data[15]->relative_position     =   transitionXYZ( 0.000 , -0.093 , -0.018 );
        op2_link_data[15]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        op2_link_data[15]->center_of_mass        =   transitionXYZ( -0.012 , 0.000 , -0.025 );
        op2_link_data[15]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[15]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[15]->inertia               =   inertiaXYZ( 0.00024 , 0.00000 , 0.00000 , 0.00101 , 0.00000 , 0.00092 );

        // right leg hip roll
        op2_link_data[17]->name                  =   "r_leg_hip_r";
        op2_link_data[17]->parent                =   15;
        op2_link_data[17]->sibling               =   -1;
        op2_link_data[17]->child                 =   19;
        op2_link_data[17]->mass                  =   1.045;
        op2_link_data[17]->relative_position     =   transitionXYZ( 0.057 , 0.000 , -0.075 );
        op2_link_data[17]->joint_axis            =   transitionXYZ( -1.0 , 0.0 , 0.0 );
        op2_link_data[17]->center_of_mass        =   transitionXYZ( -0.068 , 0.000 , 0.000 );
        op2_link_data[17]->joint_limit_max       =   0.3 * M_PI;
        op2_link_data[17]->joint_limit_min       =   -0.3 * M_PI;
        op2_link_data[17]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // right leg hip pitch
        op2_link_data[19]->name                  =   "r_leg_hip_p";
        op2_link_data[19]->parent                =   17;
        op2_link_data[19]->sibling               =   -1;
        op2_link_data[19]->child                 =   21;
        op2_link_data[19]->mass                  =   3.095;
        op2_link_data[19]->relative_position     =   transitionXYZ( -0.057 , 0.033 , 0.000 );
        op2_link_data[19]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        op2_link_data[19]->center_of_mass        =   transitionXYZ( 0.022 , 0.007 , -0.168 );
        op2_link_data[19]->joint_limit_max       =   0.4 * M_PI;
        op2_link_data[19]->joint_limit_min       =   -0.4 * M_PI;
        op2_link_data[19]->inertia               =   inertiaXYZ( 0.04329 , -0.00027 , 0.00286 , 0.04042 , 0.00203 , 0.00560 );

        // right leg knee pitch
        op2_link_data[21]->name                  =   "r_leg_kn_p";
        op2_link_data[21]->parent                =   19;
        op2_link_data[21]->sibling               =   -1;
        op2_link_data[21]->child                 =   23;
        op2_link_data[21]->mass                  =   2.401;
        op2_link_data[21]->relative_position     =   transitionXYZ( 0.000 , -0.060 , -0.300 );
        op2_link_data[21]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        op2_link_data[21]->center_of_mass        =   transitionXYZ( -0.002 , 0.066 , -0.183 );
        op2_link_data[21]->joint_limit_max       =   0.1 * M_PI;
        op2_link_data[21]->joint_limit_min       =   -0.7 * M_PI;
        op2_link_data[21]->inertia               =   inertiaXYZ( 0.01971 , -0.00031 , -0.00294 , 0.01687 , -0.00140 , 0.00574 );

        // right leg ankle pitch
        op2_link_data[23]->name                  =   "r_leg_an_p";
        op2_link_data[23]->parent                =   21;
        op2_link_data[23]->sibling               =   -1;
        op2_link_data[23]->child                 =   25;
        op2_link_data[23]->mass                  =   1.045;
        op2_link_data[23]->relative_position     =   transitionXYZ( 0.000 , 0.060 , -0.300 );
        op2_link_data[23]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[23]->center_of_mass        =   transitionXYZ( -0.011 , 0.033 , 0.000 );
        op2_link_data[23]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[23]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[23]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // right leg ankle roll
        op2_link_data[25]->name                  =   "r_leg_an_r";
        op2_link_data[25]->parent                =   23;
        op2_link_data[25]->sibling               =   -1;
        op2_link_data[25]->child                 =   37;
        op2_link_data[25]->mass                  =   0.223;
        op2_link_data[25]->relative_position     =   transitionXYZ( 0.057 , 0.033 , 0.000 );
        op2_link_data[25]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        op2_link_data[25]->center_of_mass        =   transitionXYZ( -0.070 , 0.000 , -0.048 );
        op2_link_data[25]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[25]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[25]->inertia               =   inertiaXYZ( 0.00022 , 0.00000 , -0.00001 , 0.00099 , 0.00000 , 0.00091 );

        // right leg ft
        op2_link_data[37]->name                  =   "r_leg_ft";
        op2_link_data[37]->parent                =   25;
        op2_link_data[37]->sibling               =   -1;
        op2_link_data[37]->child                 =   45;
        op2_link_data[37]->mass                  =   1.689;
        op2_link_data[37]->relative_position     =   transitionXYZ( -0.057 , 0.000 , -0.087 );
        op2_link_data[37]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[37]->center_of_mass        =   transitionXYZ( 0.000 , -0.009 , -0.013 );
        op2_link_data[37]->joint_limit_max       =   100.0;
        op2_link_data[37]->joint_limit_min       =   -100.0;
        op2_link_data[37]->inertia               =   inertiaXYZ( 0.00219 , 0.00000 , 0.00000 , 0.00433 , -0.00011 , 0.00609 );

        // right leg end
        op2_link_data[45]->name                  =   "r_leg_end";
        op2_link_data[45]->parent                =   37;
        op2_link_data[45]->sibling               =   -1;
        op2_link_data[45]->child                 =   -1;
        op2_link_data[45]->mass                  =   0.0;
        //op2_link_data[45]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.021 );
        op2_link_data[45]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.0275 );
        op2_link_data[45]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[45]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[45]->joint_limit_max       =   100.0;
        op2_link_data[45]->joint_limit_min       =   -100.0;
        op2_link_data[45]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

        /* ----- left leg -----*/

        // left leg hip yaw
        op2_link_data[16]->name                  =   "l_leg_hip_y";
        op2_link_data[16]->parent                =   44;
        op2_link_data[16]->sibling               =   -1;
        op2_link_data[16]->child                 =   18;
        op2_link_data[16]->mass                  =   0.243;
        op2_link_data[16]->relative_position     =   transitionXYZ( 0.000 , 0.093 , -0.018 );
        op2_link_data[16]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , -1.0 );
        op2_link_data[16]->center_of_mass        =   transitionXYZ( 0.012 , 0.000 , -0.025 );
        op2_link_data[16]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[16]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[16]->inertia               =   inertiaXYZ( 0.00024 , 0.00000 , 0.00000 , 0.00101 , 0.00000 , 0.00092 );

        // left leg hip roll
        op2_link_data[18]->name                  =   "l_leg_hip_r";
        op2_link_data[18]->parent                =   16;
        op2_link_data[18]->sibling               =   -1;
        op2_link_data[18]->child                 =   20;
        op2_link_data[18]->mass                  =   1.045;
        op2_link_data[18]->relative_position     =   transitionXYZ( 0.057 , 0.000 , -0.075 );
        op2_link_data[18]->joint_axis            =   transitionXYZ( -1.0 , 0.0 , 0.0 );
        op2_link_data[18]->center_of_mass        =   transitionXYZ( -0.068 , 0.000 , 0.000 );
        op2_link_data[18]->joint_limit_max       =   0.3 * M_PI;
        op2_link_data[18]->joint_limit_min       =   -0.3 * M_PI;
        op2_link_data[18]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // left leg hip pitch
        op2_link_data[20]->name                  =   "l_leg_hip_p";
        op2_link_data[20]->parent                =   18;
        op2_link_data[20]->sibling               =   -1;
        op2_link_data[20]->child                 =   22;
        op2_link_data[20]->mass                  =   3.095;
        op2_link_data[20]->relative_position     =   transitionXYZ( -0.057 , 0.033 , 0.000 );
        op2_link_data[20]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[20]->center_of_mass        =   transitionXYZ( 0.022 , -0.007 , -0.168 );
        op2_link_data[20]->joint_limit_max       =   0.4 * M_PI;
        op2_link_data[20]->joint_limit_min       =   -0.4 * M_PI;
        op2_link_data[20]->inertia               =   inertiaXYZ( 0.04328 , 0.00028 , 0.00288 , 0.04042 , -0.00202 , 0.00560 );

        // left leg knee pitch
        op2_link_data[22]->name                  =   "l_leg_kn_p";
        op2_link_data[22]->parent                =   20;
        op2_link_data[22]->sibling               =   -1;
        op2_link_data[22]->child                 =   24;
        op2_link_data[22]->mass                  =   2.401;
        op2_link_data[22]->relative_position     =   transitionXYZ( 0.000 , 0.060 , -0.300 );
        op2_link_data[22]->joint_axis            =   transitionXYZ( 0.0 , 1.0 , 0.0 );
        op2_link_data[22]->center_of_mass        =   transitionXYZ( -0.002 , -0.066 , -0.183 );
        op2_link_data[22]->joint_limit_max       =   0.7 * M_PI;
        op2_link_data[22]->joint_limit_min       =   -0.1 * M_PI;
        op2_link_data[22]->inertia               =   inertiaXYZ( 0.01971 , 0.00031 , -0.00294 , 0.01687 , 0.00140 , 0.00574 );

        // left leg ankle pitch
        op2_link_data[24]->name                  =   "l_leg_an_p";
        op2_link_data[24]->parent                =   22;
        op2_link_data[24]->sibling               =   -1;
        op2_link_data[24]->child                 =   26;
        op2_link_data[24]->mass                  =   1.045;
        op2_link_data[24]->relative_position     =   transitionXYZ( 0.000 , -0.060 , -0.300 );
        op2_link_data[24]->joint_axis            =   transitionXYZ( 0.0 , -1.0 , 0.0 );
        op2_link_data[24]->center_of_mass        =   transitionXYZ( -0.011 , -0.033 , 0.000 );
        op2_link_data[24]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[24]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[24]->inertia               =   inertiaXYZ( 0.00056 , 0.00000 , 0.00000 , 0.00168 , 0.00000 , 0.00171 );

        // left leg ankle pitch
        op2_link_data[26]->name                  =   "l_leg_an_r";
        op2_link_data[26]->parent                =   24;
        op2_link_data[26]->sibling               =   -1;
        op2_link_data[26]->child                 =   36;
        op2_link_data[26]->mass                  =   0.223;
        op2_link_data[26]->relative_position     =   transitionXYZ( 0.057 , -0.033 , 0.000 );
        op2_link_data[26]->joint_axis            =   transitionXYZ( 1.0 , 0.0 , 0.0 );
        op2_link_data[26]->center_of_mass        =   transitionXYZ( -0.070 , 0.000 , -0.048 );
        op2_link_data[26]->joint_limit_max       =   0.45 * M_PI;
        op2_link_data[26]->joint_limit_min       =   -0.45 * M_PI;
        op2_link_data[26]->inertia               =   inertiaXYZ( 0.00022 , 0.00000 , -0.00001 , 0.00099 , 0.00000 , 0.00091 );

        // left leg ft
        op2_link_data[36]->name                  =   "l_leg_ft";
        op2_link_data[36]->parent                =   26;
        op2_link_data[36]->sibling               =   -1;
        op2_link_data[36]->child                 =   46;
        op2_link_data[36]->mass                  =   0.0;
        op2_link_data[36]->relative_position     =   transitionXYZ( -0.057 , 0.000 , -0.087 );
        op2_link_data[36]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[36]->center_of_mass        =   transitionXYZ( 0.000 , 0.009 , -0.013 );
        op2_link_data[36]->joint_limit_max       =   100.0;
        op2_link_data[36]->joint_limit_min       =   -100.0;
        op2_link_data[36]->inertia               =   inertiaXYZ( 0.00219 , 0.00000 , 0.00000 , 0.00433 , 0.00011 , 0.00609 );

        // left leg end
        op2_link_data[46]->name                  =   "l_leg_end";
        op2_link_data[46]->parent                =   36;
        op2_link_data[46]->sibling               =   -1;
        op2_link_data[46]->child                 =   -1;
        op2_link_data[46]->mass                  =   0.0;
        //op2_link_data[46]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.021 );
        op2_link_data[46]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.0275 );
        op2_link_data[46]->joint_axis            =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[46]->center_of_mass        =   transitionXYZ( 0.0 , 0.0 , 0.0 );
        op2_link_data[46]->joint_limit_max       =   100.0;
        op2_link_data[46]->joint_limit_min       =   -100.0;
        op2_link_data[46]->inertia               =   inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );
    }

	thigh_length_m 		= std::fabs(op2_link_data[id_r_leg_start + 2*3]->relative_position.coeff(2,0));
	calf_length_m  		= std::fabs(op2_link_data[id_r_leg_start + 2*4]->relative_position.coeff(2,0));
	ankle_length_m 		= std::fabs(op2_link_data[id_r_leg_ft]->relative_position.coeff(2,0)
								  + op2_link_data[id_r_leg_end]->relative_position.coeff(2,0));
	leg_side_offset_m 	= 2.0*(std::fabs(op2_link_data[id_r_leg_start]->relative_position.coeff(1, 0)));

	std::cout << "thigh_length_m : " << thigh_length_m << "  calf_length_m : " << calf_length_m << " ankle_length_m : " << ankle_length_m <<std::endl;

}

std::vector<int> OP2KinematicsDynamics::findRoute( int to )
{
    int _id = op2_link_data[ to ]->parent;

    std::vector<int> _idx;

    if( _id == 0 )
    {
        _idx.push_back(0);
        _idx.push_back( to );
    }
    else
    {
        _idx = findRoute( _id );
        _idx.push_back( to );
    }

    return _idx;
}

std::vector<int> OP2KinematicsDynamics::findRoute( int from , int to )
{
    int _id = op2_link_data[ to ]->parent;

    std::vector<int> _idx;

    if( _id == from )
    {
        _idx.push_back( from );
        _idx.push_back( to );
    }
    else if ( _id != 0 )
    {
        _idx = findRoute( from , _id );
        _idx.push_back( to );
    }

    return _idx;
}

double OP2KinematicsDynamics::TotalMass( int joint_ID )
{
    double _mass;

    if ( joint_ID == -1 )
        _mass = 0.0;
    else
        _mass = op2_link_data[ joint_ID ]->mass + TotalMass( op2_link_data[ joint_ID ]->sibling ) + TotalMass( op2_link_data[ joint_ID ]->child );

    return _mass;
}

Eigen::MatrixXd OP2KinematicsDynamics::CalcMC( int joint_ID )
{
    Eigen::MatrixXd _mc(3,1);

    if ( joint_ID == -1 )
        _mc = Eigen::MatrixXd::Zero(3,1);
    else
    {
        _mc = op2_link_data[ joint_ID ]->mass * ( op2_link_data[ joint_ID ]->orientation * op2_link_data[ joint_ID ]->center_of_mass + op2_link_data[ joint_ID ]->position );
        _mc = _mc + CalcMC( op2_link_data[ joint_ID ]->sibling ) + CalcMC( op2_link_data[ joint_ID ]->child );
    }

    return _mc;
}

Eigen::MatrixXd OP2KinematicsDynamics::CalcCOM( Eigen::MatrixXd MC )
{
    double _mass ;
    Eigen::MatrixXd _COM( 3 , 1 );

    _mass = TotalMass( 0 );

    _COM = MC / _mass;

    return _COM;
}

void OP2KinematicsDynamics::ForwardKinematics( int joint_ID )
{
    if ( joint_ID == -1 )
        return;

    if ( joint_ID == 0 )
    {
        op2_link_data[0]->position       = Eigen::MatrixXd::Zero(3,1);
        op2_link_data[0]->orientation    = Rodrigues( hatto( op2_link_data[0]->joint_axis ), op2_link_data[ 0 ]->joint_angle );
    }

    if ( joint_ID != 0 )
    {
        int _parent = op2_link_data[ joint_ID ]->parent;

        op2_link_data[ joint_ID ]->position =
                op2_link_data[ _parent ]->orientation * op2_link_data[ joint_ID ]->relative_position + op2_link_data[ _parent ]->position;
        op2_link_data[ joint_ID ]->orientation =
                op2_link_data[ _parent ]->orientation * Rodrigues( hatto( op2_link_data[ joint_ID ]->joint_axis ), op2_link_data[ joint_ID ]->joint_angle );

        op2_link_data[ joint_ID ]->transformation.block<3, 1> ( 0 , 3 ) = op2_link_data[ joint_ID ]->position;
        op2_link_data[ joint_ID ]->transformation.block<3, 3> ( 0 , 0 ) = op2_link_data[ joint_ID ]->orientation;
    }

    ForwardKinematics( op2_link_data[ joint_ID ]->sibling );
    ForwardKinematics( op2_link_data[ joint_ID ]->child );
}

Eigen::MatrixXd OP2KinematicsDynamics::CalcJacobian( std::vector<int> idx )
{
    int _idx_size 	= 	idx.size();
    int _end 		= 	_idx_size - 1;

    Eigen::MatrixXd _tar_position    = 	op2_link_data[ idx[ _end ] ]->position;
    Eigen::MatrixXd _Jacobian        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++)
    {
        int _id 					= 	idx[ id ];

        Eigen::MatrixXd _tar_orientation      = 	op2_link_data[ _id ]->orientation * op2_link_data[ _id ]->joint_axis;

        _Jacobian.block( 0 , id , 3 , 1 ) 	= 	cross( _tar_orientation , _tar_position - op2_link_data[ _id ]->position );
        _Jacobian.block( 3 , id , 3 , 1 ) 	= 	_tar_orientation;
    }

    return _Jacobian;
}

Eigen::MatrixXd OP2KinematicsDynamics::CalcJacobianCOM(std::vector<int> idx)
{
    int _idx_size	=	idx.size();
    int _end 		=	_idx_size - 1;

    Eigen::MatrixXd _target_position    = 	op2_link_data[ idx[ _end ] ]->position;
    Eigen::MatrixXd _jacobianCOM        = 	Eigen::MatrixXd::Zero( 6 , _idx_size );

    for ( int id = 0; id < _idx_size; id++ )
    {
        int _id 			= 	idx[ id ];
        double _mass 	= 	TotalMass( _id );

        Eigen::MatrixXd _og                     =	CalcMC( _id ) / _mass - op2_link_data[ _id ]->position;
        Eigen::MatrixXd _target_orientation 	= 	op2_link_data[ _id ]->orientation * op2_link_data[ _id ]->joint_axis;

        _jacobianCOM.block( 0 , id , 3 , 1 ) 	= 	cross( _target_orientation , _og );
        _jacobianCOM.block( 3 , id , 3 , 1 ) 	= 	_target_orientation;
    }

    return _jacobianCOM;
}

Eigen::MatrixXd OP2KinematicsDynamics::CalcVWerr( Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation )
{
    Eigen::MatrixXd _pos_err        =	tar_position - curr_position;
//    Eigen::MatrixXd _ori_err        =	curr_orientation.inverse() * tar_orientation;
    Eigen::MatrixXd _ori_err        =	curr_orientation.transpose() * tar_orientation;
    Eigen::MatrixXd __ori_err      =	curr_orientation * rot2omega( _ori_err );

    Eigen::MatrixXd _err 	= 	Eigen::MatrixXd::Zero( 6 , 1 );
//    _err.block(0,0,3,1) 	= 	_pos_err;
//    _err.block(3,0,3,1) 	= 	__ori_err;
    _err.block<3, 1> ( 0 , 0 ) = _pos_err;
    _err.block<3, 1> ( 3 , 0 ) = __ori_err;

    return _err;
}

bool OP2KinematicsDynamics::InverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

//    ForwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	CalcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	op2_link_data[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	op2_link_data[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	CalcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	_Jacobian * _Jacobian.transpose();
        Eigen::MatrixXd ___Jacobian     = 	_Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            op2_link_data[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        ForwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( op2_link_data[ _joint_num ]->joint_angle >= op2_link_data[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( op2_link_data[ _joint_num ]->joint_angle <= op2_link_data[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool OP2KinematicsDynamics::InverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err )
{
    bool ik_success = false;
    bool limit_success = false;

//    ForwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( from , to );

    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	CalcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	op2_link_data[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	op2_link_data[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	CalcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	_Jacobian * _Jacobian.transpose();
        Eigen::MatrixXd ___Jacobian     = 	_Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            op2_link_data[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        ForwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( op2_link_data[ _joint_num ]->joint_angle >= op2_link_data[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( op2_link_data[ _joint_num ]->joint_angle <= op2_link_data[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool OP2KinematicsDynamics::InverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight )
{
    bool ik_success = false;
    bool limit_success = false;

//    ForwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( to );

    /* weight */
    Eigen::MatrixXd _weight = Eigen::MatrixXd::Identity( _idx.size() , _idx.size() );

    for ( int _ix = 0; _ix < _idx.size(); _ix++ )
        _weight.coeffRef( _ix , _ix ) = weight.coeff( _idx[ _ix ], 0 );

    /* damping */
    Eigen::MatrixXd _eval 		= 	Eigen::MatrixXd::Zero( 6 , 6 );

    double p_damping 		= 	1e-5;
    double R_damping 		= 	1e-5;

    for ( int _ix = 0; _ix < 3; _ix++ )
    {
        _eval.coeffRef( _ix , _ix )              =	p_damping;
        _eval.coeffRef( _ix + 3 , _ix + 3 ) 	=	R_damping;
    }

    /* ik */
    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	CalcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	op2_link_data[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	op2_link_data[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	CalcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;

        Eigen::MatrixXd __Jacobian      = 	( _Jacobian * _weight * _Jacobian.transpose() + _eval );
        Eigen::MatrixXd ___Jacobian     = 	_weight * _Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            op2_link_data[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        ForwardKinematics(0);
    }

    /* check joint limit */
    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( op2_link_data[ _joint_num ]->joint_angle >= op2_link_data[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( op2_link_data[ _joint_num ]->joint_angle <= op2_link_data[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool OP2KinematicsDynamics::InverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight )
{
    bool ik_success = false;
    bool limit_success = false;

//    ForwardKinematics(0);

    std::vector<int> _idx   = 	findRoute( from , to );

    /* weight */
    Eigen::MatrixXd _weight = Eigen::MatrixXd::Identity( _idx.size() , _idx.size() );

    for ( int _ix = 0; _ix < _idx.size(); _ix++ )
        _weight.coeffRef( _ix , _ix ) = weight.coeff( _idx[ _ix ], 0 );

    /* damping */
    Eigen::MatrixXd _eval 		= 	Eigen::MatrixXd::Zero( 6 , 6 );

    double p_damping 		= 	1e-5;
    double R_damping 		= 	1e-5;

    for ( int _ix = 0; _ix < 3; _ix++ )
    {
        _eval.coeffRef( _ix , _ix )              =	p_damping;
        _eval.coeffRef( _ix + 3 , _ix + 3 ) 	=	R_damping;
    }

    /* ik */
    for ( int iter = 0; iter < max_iter; iter++ )
    {
        Eigen::MatrixXd _Jacobian		= 	CalcJacobian( _idx );

        Eigen::MatrixXd _curr_position      = 	op2_link_data[ to ]->position;
        Eigen::MatrixXd _curr_orientation 	= 	op2_link_data[ to ]->orientation;

        Eigen::MatrixXd _err 	= 	CalcVWerr( tar_position,_curr_position, tar_orientation, _curr_orientation );

        if ( _err.norm() < ik_err )
        {
            ik_success = true;
            break;
        }
        else
            ik_success = false;


        Eigen::MatrixXd __Jacobian      = 	( _Jacobian * _weight * _Jacobian.transpose() + _eval );
        Eigen::MatrixXd ___Jacobian     = 	_weight * _Jacobian.transpose() * __Jacobian.inverse();

        Eigen::MatrixXd _delta_angle 		= 	___Jacobian * _err ;

        for ( int id = 0; id < _idx.size(); id++ )
        {
            int _joint_num      = 	_idx[ id ];
            op2_link_data[ _joint_num ]->joint_angle    += 	_delta_angle.coeff( id );
        }

        ForwardKinematics(0);
    }

    for ( int id = 0; id < _idx.size(); id++ )
    {
        int _joint_num      = 	_idx[ id ];

        if ( op2_link_data[ _joint_num ]->joint_angle >= op2_link_data[ _joint_num ]->joint_limit_max )
        {
            limit_success = false;
            break;
        }
        else if ( op2_link_data[ _joint_num ]->joint_angle <= op2_link_data[ _joint_num ]->joint_limit_min )
        {
            limit_success = false;
            break;
        }
        else
            limit_success = true;
    }

    if ( ik_success == true && limit_success == true )
        return true;
    else
        return false;
}

bool OP2KinematicsDynamics::InverseKinematicsforLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
	//Eigen::MatrixXd target_transform;
	Eigen::Matrix4d Tad, Tda, Tcd, Tdc, Tac;
	Eigen::Vector3d vec;

	bool  invertible;
	double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
	double THIGH_LENGTH 	= thigh_length_m;   //std::fabs(robotis_joint[id_r_leg_start + 2*3]->relative_position.coeff(2,0));
	double CALF_LENGTH  	= calf_length_m;   //std::fabs(robotis_joint[id_r_leg_start + 2*4]->relative_position.coeff(2,0));
	double ANKLE_LENGTH 	= ankle_length_m; //std::fabs(robotis_joint[id_r_leg_end]->relative_position(2,0));

	//target_transform = transformationXYZRPY(x, y, z, roll, pitch, yaw);

//
//	Tad << target_transform.coeff(0,0),  target_transform.coeff(0,1),  target_transform.coeff(0,2),  target_transform.coeff(0,3),
//		   target_transform.coeff(1,0),  target_transform.coeff(1,1),  target_transform.coeff(1,2),  target_transform.coeff(1,3),
//		   target_transform.coeff(2,0),  target_transform.coeff(2,1),  target_transform.coeff(2,2),  target_transform.coeff(2,3),
//		   target_transform.coeff(3,0),  target_transform.coeff(3,1),  target_transform.coeff(3,2),  target_transform.coeff(3,3);

	Tad = transformationXYZRPY(x, y, z, roll, pitch, yaw);

	vec.coeffRef(0) = Tad.coeff(0,3) + Tad.coeff(0,2) * ANKLE_LENGTH;
	vec.coeffRef(1) = Tad.coeff(1,3) + Tad.coeff(1,2) * ANKLE_LENGTH;
	vec.coeffRef(2) = Tad.coeff(2,3) + Tad.coeff(2,2) * ANKLE_LENGTH;

	// Get Knee
	_Rac = vec.norm();
	_Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2.0 * THIGH_LENGTH * CALF_LENGTH));
	if(std::isnan(_Acos) == 1)
		return false;
	*(out + 3) = _Acos;

	// Get Ankle Roll
	Tad.computeInverseWithCheck(Tda, invertible);
	if(invertible == false)
		return false;

	_k = sqrt(Tda.coeff(1,3) * Tda.coeff(1,3) +  Tda.coeff(2,3) * Tda.coeff(2,3));
	_l = sqrt(Tda.coeff(1,3) * Tda.coeff(1,3) + (Tda.coeff(2,3) - ANKLE_LENGTH)*(Tda.coeff(2,3) - ANKLE_LENGTH));
	_m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2.0 * _l * ANKLE_LENGTH);

	if(_m > 1.0)
		_m = 1.0;
	else if(_m < -1.0)
		_m = -1.0;
	_Acos = acos(_m);

	if(std::isnan(_Acos) == 1)
		return false;

	if(Tda.coeff(1,3) < 0.0)
		*(out + 5) = -_Acos;
	else
		*(out + 5) = _Acos;

	// Get Hip Yaw
	Tcd = transformationXYZRPY(0, 0, -ANKLE_LENGTH, *(out + 5), 0, 0);
	Tcd.computeInverseWithCheck(Tdc, invertible);
	if(invertible == false)
		return false;

	Tac = Tad * Tdc;
	_Atan = atan2(-Tac.coeff(0,1) , Tac.coeff(1,1));
	if(std::isinf(_Atan) != 0)
		return false;
	*(out) = _Atan;

	// Get Hip Roll
	_Atan = atan2(Tac.coeff(2,1), -Tac.coeff(0,1) * sin(*(out)) + Tac.coeff(1,1) * cos(*(out)));
	if(std::isinf(_Atan) != 0)
		return false;
	*(out + 1) = _Atan;


	// Get Hip Pitch and Ankle Pitch
	_Atan = atan2(Tac.coeff(0,2) * cos(*(out)) + Tac.coeff(1,2) * sin(*(out)), Tac.coeff(0,0) * cos(*(out)) + Tac.coeff(1,0) * sin(*(out)));
	if(std::isinf(_Atan) == 1)
		return false;
	_theta = _Atan;
	_k = sin(*(out + 3)) * CALF_LENGTH;
	_l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
	_m = cos(*(out)) * vec.coeff(0) + sin(*(out)) * vec.coeff(1);
	_n = cos(*(out + 1)) * vec.coeff(2) + sin(*(out)) * sin(*(out + 1)) * vec.coeff(0) - cos(*(out)) * sin(*(out + 1)) * vec.coeff(1);
	_s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
	_c = (_n - _k * _s) / _l;
	_Atan = atan2(_s, _c);
	if(std::isinf(_Atan) == 1)
		return false;
	*(out + 2) = _Atan;
	*(out + 4) = _theta - *(out + 3) - *(out + 2);


	return true;

}

bool OP2KinematicsDynamics::InverseKinematicsforRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
	if(InverseKinematicsforLeg(out, x, y, z, roll, pitch, yaw) == true) {

		*(out + 0) = out[0] * (op2_link_data[id_r_leg_start + 2*0]->joint_axis.coeff(2, 0));
		*(out + 1) = out[1] * (op2_link_data[id_r_leg_start + 2*1]->joint_axis.coeff(0, 0));
		*(out + 2) = out[2] * (op2_link_data[id_r_leg_start + 2*2]->joint_axis.coeff(1, 0));
		*(out + 3) = out[3] * (op2_link_data[id_r_leg_start + 2*3]->joint_axis.coeff(1, 0));
		*(out + 4) = out[4] * (op2_link_data[id_r_leg_start + 2*4]->joint_axis.coeff(1, 0));
		*(out + 5) = out[5] * (op2_link_data[id_r_leg_start + 2*5]->joint_axis.coeff(0, 0));
		return true;
	}
	else
		return false;
}

bool OP2KinematicsDynamics::InverseKinematicsforLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
{
	if(InverseKinematicsforLeg(out, x, y, z, roll, pitch, yaw) == true) {

		out[0] = out[0] * (op2_link_data[id_l_leg_start + 2*0]->joint_axis.coeff(2, 0));
		out[1] = out[1] * (op2_link_data[id_l_leg_start + 2*1]->joint_axis.coeff(0, 0));
		out[2] = out[2] * (op2_link_data[id_l_leg_start + 2*2]->joint_axis.coeff(1, 0));
		out[3] = out[3] * (op2_link_data[id_l_leg_start + 2*3]->joint_axis.coeff(1, 0));
		out[4] = out[4] * (op2_link_data[id_l_leg_start + 2*4]->joint_axis.coeff(1, 0));
		out[5] = out[5] * (op2_link_data[id_l_leg_start + 2*5]->joint_axis.coeff(0, 0));
		return true;
	}
	else
		return false;
}

}
