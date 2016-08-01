/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include <iostream>

namespace ROBOTIS
{

OP3KinematicsDynamics::OP3KinematicsDynamics()
{
}
OP3KinematicsDynamics::~OP3KinematicsDynamics()
{
}

OP3KinematicsDynamics::OP3KinematicsDynamics(TREE_SELECT tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    op3_link_data[id] = new LinkData();

  // Todo : Make tree for OP3
  if (tree == WHOLE_BODY)
  {
    op3_link_data[0]->name = "base";
    op3_link_data[0]->parent = -1;
    op3_link_data[0]->sibling = -1;
    op3_link_data[0]->child = 23;
    op3_link_data[0]->mass = 0.0;
    op3_link_data[0]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[0]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[0]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[0]->joint_limit_max = 100.0;
    op3_link_data[0]->joint_limit_min = -100.0;
    op3_link_data[0]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- passive joint -----*/

    op3_link_data[23]->name = "passive_x";
    op3_link_data[23]->parent = 0;
    op3_link_data[23]->sibling = -1;
    op3_link_data[23]->child = 24;
    op3_link_data[23]->mass = 0.0;
    op3_link_data[23]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[23]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[23]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[23]->joint_limit_max = 100.0;
    op3_link_data[23]->joint_limit_min = -100.0;
    op3_link_data[23]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data[24]->name = "passive_y";
    op3_link_data[24]->parent = 23;
    op3_link_data[24]->sibling = -1;
    op3_link_data[24]->child = 25;
    op3_link_data[24]->mass = 0.0;
    op3_link_data[24]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[24]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[24]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[24]->joint_limit_max = 100.0;
    op3_link_data[24]->joint_limit_min = -100.0;
    op3_link_data[24]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data[25]->name = "passive_z";
    op3_link_data[25]->parent = 24;
    op3_link_data[25]->sibling = -1;
    op3_link_data[25]->child = 26;
    op3_link_data[25]->mass = 0.0;
    op3_link_data[25]->relative_position = transitionXYZ(0.0, 0.0, 0.801);
    op3_link_data[25]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[25]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[25]->joint_limit_max = 100.0;
    op3_link_data[25]->joint_limit_min = -100.0;
    op3_link_data[25]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data[26]->name = "passive_roll";
    op3_link_data[26]->parent = 25;
    op3_link_data[26]->sibling = -1;
    op3_link_data[26]->child = 27;
    op3_link_data[26]->mass = 0.0;
    op3_link_data[26]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[26]->joint_axis = transitionXYZ(1.0, 0.0, 0.0);
    op3_link_data[26]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[26]->joint_limit_max = 100.0;
    op3_link_data[26]->joint_limit_min = -100.0;
    op3_link_data[26]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data[27]->name = "passive_pitch";
    op3_link_data[27]->parent = 26;
    op3_link_data[27]->sibling = -1;
    op3_link_data[27]->child = 28;
    op3_link_data[27]->mass = 0.0;
    op3_link_data[27]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[27]->joint_axis = transitionXYZ(0.0, 1.0, 0.0);
    op3_link_data[27]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[27]->joint_limit_max = 100.0;
    op3_link_data[27]->joint_limit_min = -100.0;
    op3_link_data[27]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    op3_link_data[28]->name = "passive_yaw";
    op3_link_data[28]->parent = 27;
    op3_link_data[28]->sibling = -1;
    op3_link_data[28]->child = 29;
    op3_link_data[28]->mass = 0.0;
    op3_link_data[28]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[28]->joint_axis = transitionXYZ(0.0, 0.0, 1.0);
    op3_link_data[28]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[28]->joint_limit_max = 100.0;
    op3_link_data[28]->joint_limit_min = -100.0;
    op3_link_data[28]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- body -----*/

    // pelvis_link
    op3_link_data[29]->name = "pelvis";
    op3_link_data[29]->parent = 28;
    op3_link_data[29]->sibling = -1;
    op3_link_data[29]->child = 19;
    op3_link_data[29]->mass = 6.869;
    op3_link_data[29]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[29]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[29]->center_of_mass = transitionXYZ(-0.011, 0.000, 0.058);
    op3_link_data[29]->joint_limit_max = 100.0;
    op3_link_data[29]->joint_limit_min = -100.0;
    op3_link_data[29]->inertia = inertiaXYZ(0.03603, 0.00000, 0.00016, 0.02210, 0.00000, 0.03830);

    /* ----- head -----*/

    // head_pan
    op3_link_data[19]->name = "head_pan";
    op3_link_data[19]->parent = 29;
    op3_link_data[19]->sibling = 1;
    op3_link_data[19]->child = 20;
    op3_link_data[19]->mass = 0.087;
    op3_link_data[19]->relative_position = transitionXYZ(0.0, 0.0, 0.0205);
    op3_link_data[19]->joint_axis = transitionXYZ(0.0, 0.0, 1.0);
    op3_link_data[19]->center_of_mass = transitionXYZ(0.000, -0.002, 0.010);
    op3_link_data[19]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[19]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[19]->inertia = inertiaXYZ(0.00011, 0.00000, 0.00000, 0.00003, 0.00000, 0.00012);

    // head_tilt
    op3_link_data[20]->name = "head_tilt";
    op3_link_data[20]->parent = 19;
    op3_link_data[20]->sibling = -1;
    op3_link_data[20]->child = -1;
    op3_link_data[20]->mass = 0.724;
    op3_link_data[20]->relative_position = transitionXYZ(0.0, 0.0, 0.03);
    op3_link_data[20]->joint_axis = transitionXYZ(0.0, -1.0, 0.0);
    op3_link_data[20]->center_of_mass = transitionXYZ(0.009, 0.046, 0.022);
    op3_link_data[20]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[20]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[20]->inertia = inertiaXYZ(0.00113, 0.00001, -0.00005, 0.00114, 0.00002, 0.00084);

    /*----- right arm -----*/

    // right arm shoulder pitch
    op3_link_data[1]->name = "r_sho_pitch";
    op3_link_data[1]->parent = 29;
    op3_link_data[1]->sibling = 2;
    op3_link_data[1]->child = 3;
    op3_link_data[1]->mass = 0.194;
    op3_link_data[1]->relative_position = transitionXYZ(0.0, -0.0575, 0.0);
    op3_link_data[1]->joint_axis = transitionXYZ(0.0, -1.0, 0.0);
    op3_link_data[1]->center_of_mass = transitionXYZ(-0.003, -0.020, -0.005);
    op3_link_data[1]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[1]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[1]->inertia = inertiaXYZ(0.00018, 0.0, 0.0, 0.00058, -0.00004, 0.00057);

    // right arm shoulder roll
    op3_link_data[3]->name = "r_sho_roll";
    op3_link_data[3]->parent = 1;
    op3_link_data[3]->sibling = -1;
    op3_link_data[3]->child = 5;
    op3_link_data[3]->mass = 0.875;
    op3_link_data[3]->relative_position = transitionXYZ(0.0, -0.0245, -0.016);
    op3_link_data[3]->joint_axis = transitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data[3]->center_of_mass = transitionXYZ(-0.060, -0.002, 0.000);
    op3_link_data[3]->joint_limit_max = 0.3 * M_PI;
    op3_link_data[3]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[3]->inertia = inertiaXYZ(0.00043, 0.00000, 0.00000, 0.00112, 0.00000, 0.00113);

    // right arm elbow
    op3_link_data[5]->name = "r_el";
    op3_link_data[5]->parent = 3;
    op3_link_data[5]->sibling = -1;
    op3_link_data[5]->child = 21;
    op3_link_data[5]->mass = 1.122;
    op3_link_data[5]->relative_position = transitionXYZ(0.016, 0.0, -0.06);
    op3_link_data[5]->joint_axis = transitionXYZ(0.0, -1.0, 0.0);
    op3_link_data[5]->center_of_mass = transitionXYZ(0.000, -0.073, 0.000);
    op3_link_data[5]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[5]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[5]->inertia = inertiaXYZ(0.00277, 0.00002, -0.00001, 0.00090, 0.00004, 0.00255);

    // right arm end effector
    op3_link_data[21]->name = "r_arm_end";
    op3_link_data[21]->parent = 5;
    op3_link_data[21]->sibling = -1;
    op3_link_data[21]->child = -1;
    op3_link_data[21]->mass = 0.0;
    op3_link_data[21]->relative_position = transitionXYZ(0.0, 0.0, -0.06);
    op3_link_data[21]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[21]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[21]->joint_limit_max = 100.0;
    op3_link_data[21]->joint_limit_min = -100.0;
    op3_link_data[21]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /*----- left arm -----*/

    // left arm shoulder pitch
    op3_link_data[2]->name = "l_sho_pitch";
    op3_link_data[2]->parent = 27;
    op3_link_data[2]->sibling = -1;
    op3_link_data[2]->child = 4;
    op3_link_data[2]->mass = 0.194;
    op3_link_data[2]->relative_position = transitionXYZ(0.0, 0.0575, 0.0);
    op3_link_data[2]->joint_axis = transitionXYZ(0.0, 1.0, 0.0);
    op3_link_data[2]->center_of_mass = transitionXYZ(-0.003, 0.020, -0.005);
    op3_link_data[2]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[2]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[2]->inertia = inertiaXYZ(0.00018, 0.00000, 0.00000, 0.00058, 0.00004, 0.00057);

    // left arm shoulder roll
    op3_link_data[4]->name = "l_sho_roll";
    op3_link_data[4]->parent = 2;
    op3_link_data[4]->sibling = -1;
    op3_link_data[4]->child = 6;
    op3_link_data[4]->mass = 0.875;
    op3_link_data[4]->relative_position = transitionXYZ(0.0, 0.0245, -0.016);
    op3_link_data[4]->joint_axis = transitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data[4]->center_of_mass = transitionXYZ(-0.060, 0.002, 0.000);
    op3_link_data[4]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[4]->joint_limit_min = -0.3 * M_PI;
    op3_link_data[4]->inertia = inertiaXYZ(0.00043, 0.00000, 0.00000, 0.00112, 0.00000, 0.00113);

    // left arm elbow
    op3_link_data[6]->name = "l_el";
    op3_link_data[6]->parent = 4;
    op3_link_data[6]->sibling = -1;
    op3_link_data[6]->child = 22;
    op3_link_data[6]->mass = 1.122;
    op3_link_data[6]->relative_position = transitionXYZ(0.016, 0.0, -0.06);
    op3_link_data[6]->joint_axis = transitionXYZ(0.0, 1.0, 0.0);
    op3_link_data[6]->center_of_mass = transitionXYZ(0.000, 0.073, 0.000);
    op3_link_data[6]->joint_limit_max = 0.5 * M_PI;
    op3_link_data[6]->joint_limit_min = -0.5 * M_PI;
    op3_link_data[6]->inertia = inertiaXYZ(0.00277, -0.00002, -0.00001, 0.00090, -0.00004, 0.00255);

    // left arm end effector
    op3_link_data[22]->name = "l_arm_end";
    op3_link_data[22]->parent = 6;
    op3_link_data[22]->sibling = -1;
    op3_link_data[22]->child = -1;
    op3_link_data[22]->mass = 0.0;
    op3_link_data[22]->relative_position = transitionXYZ(0.0, 0.0, -0.06);
    op3_link_data[22]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[22]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[22]->joint_limit_max = 100.0;
    op3_link_data[22]->joint_limit_min = -100.0;
    op3_link_data[22]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- right leg -----*/

    // right leg hip yaw
    op3_link_data[7]->name = "r_hip_yaw";
    op3_link_data[7]->parent = 27;
    op3_link_data[7]->sibling = 8;
    op3_link_data[7]->child = 9;
    op3_link_data[7]->mass = 0.243;
    op3_link_data[7]->relative_position = transitionXYZ(-0.005, -0.035, -0.0907);
    op3_link_data[7]->joint_axis = transitionXYZ(0.0, 0.0, -1.0);
    op3_link_data[7]->center_of_mass = transitionXYZ(-0.012, 0.000, -0.025);
    op3_link_data[7]->joint_limit_max = 0.45 * M_PI;
    op3_link_data[7]->joint_limit_min = -0.45 * M_PI;
    op3_link_data[7]->inertia = inertiaXYZ(0.00024, 0.00000, 0.00000, 0.00101, 0.00000, 0.00092);

    // right leg hip roll
    op3_link_data[9]->name = "r_hip_roll";
    op3_link_data[9]->parent = 7;
    op3_link_data[9]->sibling = -1;
    op3_link_data[9]->child = 11;
    op3_link_data[9]->mass = 1.045;
    // op3_link_data[9]->relative_position     =   transitionXYZ( 0.000 , 0.000 , -0.0315 );
    op3_link_data[9]->relative_position = transitionXYZ(0.000, 0.000, -0.0285);
    op3_link_data[9]->joint_axis = transitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data[9]->center_of_mass = transitionXYZ(-0.068, 0.000, 0.000);
    op3_link_data[9]->joint_limit_max = 0.3 * M_PI;
    op3_link_data[9]->joint_limit_min = -0.3 * M_PI;
    op3_link_data[9]->inertia = inertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000, 0.00171);

    // right leg hip pitch
    op3_link_data[11]->name = "r_hip_pitch";
    op3_link_data[11]->parent = 9;
    op3_link_data[11]->sibling = -1;
    op3_link_data[11]->child = 13;
    op3_link_data[11]->mass = 3.095;
    op3_link_data[11]->relative_position = transitionXYZ(0.000, 0.000, 0.000);
    op3_link_data[11]->joint_axis = transitionXYZ(0.0, 1.0, 0.0);
    op3_link_data[11]->center_of_mass = transitionXYZ(0.022, 0.007, -0.168);
    op3_link_data[11]->joint_limit_max = 0.4 * M_PI;
    op3_link_data[11]->joint_limit_min = -0.4 * M_PI;
    op3_link_data[11]->inertia = inertiaXYZ(0.04329, -0.00027, 0.00286, 0.04042, 0.00203, 0.00560);

    // right leg knee
    op3_link_data[13]->name = "r_knee";
    op3_link_data[13]->parent = 11;
    op3_link_data[13]->sibling = -1;
    op3_link_data[13]->child = 15;
    op3_link_data[13]->mass = 2.401;
    // op3_link_data[13]->relative_position     =   transitionXYZ( 0.000 , 0.000 , -0.093 );
    op3_link_data[13]->relative_position = transitionXYZ(0.000, 0.000, -0.110);
    op3_link_data[13]->joint_axis = transitionXYZ(0.0, 1.0, 0.0);
    op3_link_data[13]->center_of_mass = transitionXYZ(-0.002, 0.066, -0.183);
    op3_link_data[13]->joint_limit_max = 0.1 * M_PI;
    op3_link_data[13]->joint_limit_min = -0.7 * M_PI;
    op3_link_data[13]->inertia = inertiaXYZ(0.01971, -0.00031, -0.00294, 0.01687, -0.00140, 0.00574);

    // right leg ankle pitch
    op3_link_data[15]->name = "r_ank_pitch";
    op3_link_data[15]->parent = 13;
    op3_link_data[15]->sibling = -1;
    op3_link_data[15]->child = 17;
    op3_link_data[15]->mass = 1.045;
    // op3_link_data[15]->relative_position     =   transitionXYZ( 0.000 , 0.000 , -0.093 );
    op3_link_data[15]->relative_position = transitionXYZ(0.000, 0.000, -0.110);
    op3_link_data[15]->joint_axis = transitionXYZ(0.0, -1.0, 0.0);
    op3_link_data[15]->center_of_mass = transitionXYZ(-0.011, 0.033, 0.000);
    op3_link_data[15]->joint_limit_max = 0.45 * M_PI;
    op3_link_data[15]->joint_limit_min = -0.45 * M_PI;
    op3_link_data[15]->inertia = inertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000, 0.00171);

    // right leg ankle roll
    op3_link_data[17]->name = "r_ank_roll";
    op3_link_data[17]->parent = 15;
    op3_link_data[17]->sibling = -1;
    op3_link_data[17]->child = 31;
    op3_link_data[17]->mass = 0.223;
    op3_link_data[17]->relative_position = transitionXYZ(0.000, 0.000, 0.000);
    op3_link_data[17]->joint_axis = transitionXYZ(1.0, 0.0, 0.0);
    op3_link_data[17]->center_of_mass = transitionXYZ(-0.070, 0.000, -0.048);
    op3_link_data[17]->joint_limit_max = 0.45 * M_PI;
    op3_link_data[17]->joint_limit_min = -0.45 * M_PI;
    op3_link_data[17]->inertia = inertiaXYZ(0.00022, 0.00000, -0.00001, 0.00099, 0.00000, 0.00091);

    // right leg end
    op3_link_data[31]->name = "r_leg_end";
    op3_link_data[31]->parent = 17;
    op3_link_data[31]->sibling = -1;
    op3_link_data[31]->child = -1;
    op3_link_data[31]->mass = 0.0;
    // op3_link_data[31]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.03 );
    op3_link_data[31]->relative_position = transitionXYZ(0.0, 0.0, -0.0305);
    op3_link_data[31]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[31]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[31]->joint_limit_max = 100.0;
    op3_link_data[31]->joint_limit_min = -100.0;
    op3_link_data[31]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /* ----- left leg -----*/

    // left leg hip yaw
    op3_link_data[8]->name = "l_hip_yaw";
    op3_link_data[8]->parent = 27;
    op3_link_data[8]->sibling = -1;
    op3_link_data[8]->child = 10;
    op3_link_data[8]->mass = 0.243;
    op3_link_data[8]->relative_position = transitionXYZ(-0.005, 0.035, -0.0907);
    op3_link_data[8]->joint_axis = transitionXYZ(0.0, 0.0, -1.0);
    op3_link_data[8]->center_of_mass = transitionXYZ(0.012, 0.000, -0.025);
    op3_link_data[8]->joint_limit_max = 0.45 * M_PI;
    op3_link_data[8]->joint_limit_min = -0.45 * M_PI;
    op3_link_data[8]->inertia = inertiaXYZ(0.00024, 0.00000, 0.00000, 0.00101, 0.00000, 0.00092);

    // left leg hip roll
    op3_link_data[10]->name = "l_hip_roll";
    op3_link_data[10]->parent = 8;
    op3_link_data[10]->sibling = -1;
    op3_link_data[10]->child = 12;
    op3_link_data[10]->mass = 1.045;
    //op3_link_data[10]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.0315 );
    op3_link_data[10]->relative_position = transitionXYZ(0.0, 0.0, -0.0285);
    op3_link_data[10]->joint_axis = transitionXYZ(-1.0, 0.0, 0.0);
    op3_link_data[10]->center_of_mass = transitionXYZ(-0.068, 0.000, 0.000);
    op3_link_data[10]->joint_limit_max = 0.3 * M_PI;
    op3_link_data[10]->joint_limit_min = -0.3 * M_PI;
    op3_link_data[10]->inertia = inertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000, 0.00171);

    // left leg hip pitch
    op3_link_data[12]->name = "l_hip_pitch";
    op3_link_data[12]->parent = 10;
    op3_link_data[12]->sibling = -1;
    op3_link_data[12]->child = 14;
    op3_link_data[12]->mass = 3.095;
    op3_link_data[12]->relative_position = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[12]->joint_axis = transitionXYZ(0.0, -1.0, 0.0);
    op3_link_data[12]->center_of_mass = transitionXYZ(0.022, -0.007, -0.168);
    op3_link_data[12]->joint_limit_max = 0.4 * M_PI;
    op3_link_data[12]->joint_limit_min = -0.4 * M_PI;
    op3_link_data[12]->inertia = inertiaXYZ(0.04328, 0.00028, 0.00288, 0.04042, -0.00202, 0.00560);

    // left leg knee pitch
    op3_link_data[14]->name = "l_knee";
    op3_link_data[14]->parent = 12;
    op3_link_data[14]->sibling = -1;
    op3_link_data[14]->child = 16;
    op3_link_data[14]->mass = 2.401;
    // op3_link_data[14]->relative_position     =   transitionXYZ( 0.000 , 0.000 , -0.093 );
    op3_link_data[14]->relative_position = transitionXYZ(0.000, 0.000, -0.110);
    op3_link_data[14]->joint_axis = transitionXYZ(0.0, -1.0, 0.0);
    op3_link_data[14]->center_of_mass = transitionXYZ(-0.002, -0.066, -0.183);
    op3_link_data[14]->joint_limit_max = 0.7 * M_PI;
    op3_link_data[14]->joint_limit_min = -0.1 * M_PI;
    op3_link_data[14]->inertia = inertiaXYZ(0.01971, 0.00031, -0.00294, 0.01687, 0.00140, 0.00574);

    // left leg ankle pitch
    op3_link_data[16]->name = "l_ank_pitch";
    op3_link_data[16]->parent = 14;
    op3_link_data[16]->sibling = -1;
    op3_link_data[16]->child = 18;
    op3_link_data[16]->mass = 1.045;
    // op3_link_data[16]->relative_position     =   transitionXYZ( 0.000 , 0.000 , -0.093 );
    op3_link_data[16]->relative_position = transitionXYZ(0.000, 0.000, -0.110);
    op3_link_data[16]->joint_axis = transitionXYZ(0.0, 1.0, 0.0);
    op3_link_data[16]->center_of_mass = transitionXYZ(-0.011, -0.033, 0.000);
    op3_link_data[16]->joint_limit_max = 0.45 * M_PI;
    op3_link_data[16]->joint_limit_min = -0.45 * M_PI;
    op3_link_data[16]->inertia = inertiaXYZ(0.00056, 0.00000, 0.00000, 0.00168, 0.00000, 0.00171);

    // left leg ankle roll
    op3_link_data[18]->name = "l_ank_roll";
    op3_link_data[18]->parent = 16;
    op3_link_data[18]->sibling = -1;
    op3_link_data[18]->child = 30;
    op3_link_data[18]->mass = 0.223;
    op3_link_data[18]->relative_position = transitionXYZ(0.000, 0.000, 0.000);
    op3_link_data[18]->joint_axis = transitionXYZ(1.0, 0.0, 0.0);
    op3_link_data[18]->center_of_mass = transitionXYZ(-0.070, 0.000, -0.048);
    op3_link_data[18]->joint_limit_max = 0.45 * M_PI;
    op3_link_data[18]->joint_limit_min = -0.45 * M_PI;
    op3_link_data[18]->inertia = inertiaXYZ(0.00022, 0.00000, -0.00001, 0.00099, 0.00000, 0.00091);

    // left leg end
    op3_link_data[30]->name = "l_leg_end";
    op3_link_data[30]->parent = 18;
    op3_link_data[30]->sibling = -1;
    op3_link_data[30]->child = -1;
    op3_link_data[30]->mass = 0.0;
    // op3_link_data[30]->relative_position     =   transitionXYZ( 0.0 , 0.0 , -0.03 );
    op3_link_data[30]->relative_position = transitionXYZ(0.0, 0.0, -0.0305);
    op3_link_data[30]->joint_axis = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[30]->center_of_mass = transitionXYZ(0.0, 0.0, 0.0);
    op3_link_data[30]->joint_limit_max = 100.0;
    op3_link_data[30]->joint_limit_min = -100.0;
    op3_link_data[30]->inertia = inertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  thigh_length_m = std::fabs(op3_link_data[id_r_leg_start + 2 * 3]->relative_position.coeff(2, 0));
  calf_length_m = std::fabs(op3_link_data[id_r_leg_start + 2 * 4]->relative_position.coeff(2, 0));
  ankle_length_m = std::fabs(op3_link_data[id_r_leg_end]->relative_position.coeff(2, 0));
  leg_side_offset_m = 2.0 * (std::fabs(op3_link_data[id_r_leg_start]->relative_position.coeff(1, 0)));

  std::cout << "thigh_length_m : " << thigh_length_m << "  calf_length_m : " << calf_length_m << " ankle_length_m : "
            << ankle_length_m << std::endl;

}

std::vector<int> OP3KinematicsDynamics::findRoute(int to)
{
  int _id = op3_link_data[to]->parent;

  std::vector<int> _idx;

  if (_id == 0)
  {
    _idx.push_back(0);
    _idx.push_back(to);
  }
  else
  {
    _idx = findRoute(_id);
    _idx.push_back(to);
  }

  return _idx;
}

std::vector<int> OP3KinematicsDynamics::findRoute(int from, int to)
{
  int _id = op3_link_data[to]->parent;

  std::vector<int> _idx;

  if (_id == from)
  {
    _idx.push_back(from);
    _idx.push_back(to);
  }
  else if (_id != 0)
  {
    _idx = findRoute(from, _id);
    _idx.push_back(to);
  }

  return _idx;
}

double OP3KinematicsDynamics::TotalMass(int joint_ID)
{
  double _mass;

  if (joint_ID == -1)
    _mass = 0.0;
  else
    _mass = op3_link_data[joint_ID]->mass + TotalMass(op3_link_data[joint_ID]->sibling)
        + TotalMass(op3_link_data[joint_ID]->child);

  return _mass;
}

Eigen::MatrixXd OP3KinematicsDynamics::CalcMC(int joint_ID)
{
  Eigen::MatrixXd _mc(3, 1);

  if (joint_ID == -1)
    _mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    _mc = op3_link_data[joint_ID]->mass
        * (op3_link_data[joint_ID]->orientation * op3_link_data[joint_ID]->center_of_mass
            + op3_link_data[joint_ID]->position);
    _mc = _mc + CalcMC(op3_link_data[joint_ID]->sibling) + CalcMC(op3_link_data[joint_ID]->child);
  }

  return _mc;
}

Eigen::MatrixXd OP3KinematicsDynamics::CalcCOM(Eigen::MatrixXd MC)
{
  double _mass;
  Eigen::MatrixXd _COM(3, 1);

  _mass = TotalMass(0);

  _COM = MC / _mass;

  return _COM;
}

void OP3KinematicsDynamics::ForwardKinematics(int joint_ID)
{
  if (joint_ID == -1)
    return;

  if (joint_ID == 0)
  {
    op3_link_data[0]->position = Eigen::MatrixXd::Zero(3, 1);
    op3_link_data[0]->orientation = Rodrigues(hatto(op3_link_data[0]->joint_axis), op3_link_data[0]->joint_angle);
  }

  if (joint_ID != 0)
  {
    int _parent = op3_link_data[joint_ID]->parent;

    op3_link_data[joint_ID]->position = op3_link_data[_parent]->orientation * op3_link_data[joint_ID]->relative_position
        + op3_link_data[_parent]->position;
    op3_link_data[joint_ID]->orientation = op3_link_data[_parent]->orientation
        * Rodrigues(hatto(op3_link_data[joint_ID]->joint_axis), op3_link_data[joint_ID]->joint_angle);

    op3_link_data[joint_ID]->transformation.block<3, 1>(0, 3) = op3_link_data[joint_ID]->position;
    op3_link_data[joint_ID]->transformation.block<3, 3>(0, 0) = op3_link_data[joint_ID]->orientation;
  }

  ForwardKinematics(op3_link_data[joint_ID]->sibling);
  ForwardKinematics(op3_link_data[joint_ID]->child);
}

Eigen::MatrixXd OP3KinematicsDynamics::CalcJacobian(std::vector<int> idx)
{
  int _idx_size = idx.size();
  int _end = _idx_size - 1;

  Eigen::MatrixXd _tar_position = op3_link_data[idx[_end]]->position;
  Eigen::MatrixXd _Jacobian = Eigen::MatrixXd::Zero(6, _idx_size);

  for (int id = 0; id < _idx_size; id++)
  {
    int _id = idx[id];

    Eigen::MatrixXd _tar_orientation = op3_link_data[_id]->orientation * op3_link_data[_id]->joint_axis;

    _Jacobian.block(0, id, 3, 1) = cross(_tar_orientation, _tar_position - op3_link_data[_id]->position);
    _Jacobian.block(3, id, 3, 1) = _tar_orientation;
  }

  return _Jacobian;
}

Eigen::MatrixXd OP3KinematicsDynamics::CalcJacobianCOM(std::vector<int> idx)
{
  int _idx_size = idx.size();
  int _end = _idx_size - 1;

  Eigen::MatrixXd _target_position = op3_link_data[idx[_end]]->position;
  Eigen::MatrixXd _jacobianCOM = Eigen::MatrixXd::Zero(6, _idx_size);

  for (int id = 0; id < _idx_size; id++)
  {
    int _id = idx[id];
    double _mass = TotalMass(_id);

    Eigen::MatrixXd _og = CalcMC(_id) / _mass - op3_link_data[_id]->position;
    Eigen::MatrixXd _target_orientation = op3_link_data[_id]->orientation * op3_link_data[_id]->joint_axis;

    _jacobianCOM.block(0, id, 3, 1) = cross(_target_orientation, _og);
    _jacobianCOM.block(3, id, 3, 1) = _target_orientation;
  }

  return _jacobianCOM;
}

Eigen::MatrixXd OP3KinematicsDynamics::CalcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                                                 Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd _pos_err = tar_position - curr_position;
//    Eigen::MatrixXd _ori_err        =	curr_orientation.inverse() * tar_orientation;
  Eigen::MatrixXd _ori_err = curr_orientation.transpose() * tar_orientation;
  Eigen::MatrixXd __ori_err = curr_orientation * rot2omega(_ori_err);

  Eigen::MatrixXd _err = Eigen::MatrixXd::Zero(6, 1);
//    _err.block(0,0,3,1) 	= 	_pos_err;
//    _err.block(3,0,3,1) 	= 	__ori_err;
  _err.block<3, 1>(0, 0) = _pos_err;
  _err.block<3, 1>(3, 0) = __ori_err;

  return _err;
}

bool OP3KinematicsDynamics::InverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                                              int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

//    ForwardKinematics(0);

  std::vector<int> _idx = findRoute(to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd _Jacobian = CalcJacobian(_idx);

    Eigen::MatrixXd _curr_position = op3_link_data[to]->position;
    Eigen::MatrixXd _curr_orientation = op3_link_data[to]->orientation;

    Eigen::MatrixXd _err = CalcVWerr(tar_position, _curr_position, tar_orientation, _curr_orientation);

    if (_err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd __Jacobian = _Jacobian * _Jacobian.transpose();
    Eigen::MatrixXd ___Jacobian = _Jacobian.transpose() * __Jacobian.inverse();

    Eigen::MatrixXd _delta_angle = ___Jacobian * _err;

    for (int id = 0; id < _idx.size(); id++)
    {
      int _joint_num = _idx[id];
      op3_link_data[_joint_num]->joint_angle += _delta_angle.coeff(id);
    }

    ForwardKinematics(0);
  }

  for (int id = 0; id < _idx.size(); id++)
  {
    int _joint_num = _idx[id];

    if (op3_link_data[_joint_num]->joint_angle >= op3_link_data[_joint_num]->joint_limit_max)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data[_joint_num]->joint_angle <= op3_link_data[_joint_num]->joint_limit_min)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool OP3KinematicsDynamics::InverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
                                              Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

//    ForwardKinematics(0);

  std::vector<int> _idx = findRoute(from, to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd _Jacobian = CalcJacobian(_idx);

    Eigen::MatrixXd _curr_position = op3_link_data[to]->position;
    Eigen::MatrixXd _curr_orientation = op3_link_data[to]->orientation;

    Eigen::MatrixXd _err = CalcVWerr(tar_position, _curr_position, tar_orientation, _curr_orientation);

    if (_err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd __Jacobian = _Jacobian * _Jacobian.transpose();
    Eigen::MatrixXd ___Jacobian = _Jacobian.transpose() * __Jacobian.inverse();

    Eigen::MatrixXd _delta_angle = ___Jacobian * _err;

    for (int id = 0; id < _idx.size(); id++)
    {
      int _joint_num = _idx[id];
      op3_link_data[_joint_num]->joint_angle += _delta_angle.coeff(id);
    }

    ForwardKinematics(0);
  }

  for (int id = 0; id < _idx.size(); id++)
  {
    int _joint_num = _idx[id];

    if (op3_link_data[_joint_num]->joint_angle >= op3_link_data[_joint_num]->joint_limit_max)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data[_joint_num]->joint_angle <= op3_link_data[_joint_num]->joint_limit_min)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool OP3KinematicsDynamics::InverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                                              int max_iter, double ik_err, Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

//    ForwardKinematics(0);

  std::vector<int> _idx = findRoute(to);

  /* weight */
  Eigen::MatrixXd _weight = Eigen::MatrixXd::Identity(_idx.size(), _idx.size());

  for (int _ix = 0; _ix < _idx.size(); _ix++)
    _weight.coeffRef(_ix, _ix) = weight.coeff(_idx[_ix], 0);

  /* damping */
  Eigen::MatrixXd _eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int _ix = 0; _ix < 3; _ix++)
  {
    _eval.coeffRef(_ix, _ix) = p_damping;
    _eval.coeffRef(_ix + 3, _ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd _Jacobian = CalcJacobian(_idx);

    Eigen::MatrixXd _curr_position = op3_link_data[to]->position;
    Eigen::MatrixXd _curr_orientation = op3_link_data[to]->orientation;

    Eigen::MatrixXd _err = CalcVWerr(tar_position, _curr_position, tar_orientation, _curr_orientation);

    if (_err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd __Jacobian = (_Jacobian * _weight * _Jacobian.transpose() + _eval);
    Eigen::MatrixXd ___Jacobian = _weight * _Jacobian.transpose() * __Jacobian.inverse();

    Eigen::MatrixXd _delta_angle = ___Jacobian * _err;

    for (int id = 0; id < _idx.size(); id++)
    {
      int _joint_num = _idx[id];
      op3_link_data[_joint_num]->joint_angle += _delta_angle.coeff(id);
    }

    ForwardKinematics(0);
  }

  /* check joint limit */
  for (int id = 0; id < _idx.size(); id++)
  {
    int _joint_num = _idx[id];

    if (op3_link_data[_joint_num]->joint_angle >= op3_link_data[_joint_num]->joint_limit_max)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data[_joint_num]->joint_angle <= op3_link_data[_joint_num]->joint_limit_min)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool OP3KinematicsDynamics::InverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
                                              Eigen::MatrixXd tar_orientation, int max_iter, double ik_err,
                                              Eigen::MatrixXd weight)
{
  bool ik_success = false;
  bool limit_success = false;

//    ForwardKinematics(0);

  std::vector<int> _idx = findRoute(from, to);

  /* weight */
  Eigen::MatrixXd _weight = Eigen::MatrixXd::Identity(_idx.size(), _idx.size());

  for (int _ix = 0; _ix < _idx.size(); _ix++)
    _weight.coeffRef(_ix, _ix) = weight.coeff(_idx[_ix], 0);

  /* damping */
  Eigen::MatrixXd _eval = Eigen::MatrixXd::Zero(6, 6);

  double p_damping = 1e-5;
  double R_damping = 1e-5;

  for (int _ix = 0; _ix < 3; _ix++)
  {
    _eval.coeffRef(_ix, _ix) = p_damping;
    _eval.coeffRef(_ix + 3, _ix + 3) = R_damping;
  }

  /* ik */
  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd _Jacobian = CalcJacobian(_idx);

    Eigen::MatrixXd _curr_position = op3_link_data[to]->position;
    Eigen::MatrixXd _curr_orientation = op3_link_data[to]->orientation;

    Eigen::MatrixXd _err = CalcVWerr(tar_position, _curr_position, tar_orientation, _curr_orientation);

    if (_err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd __Jacobian = (_Jacobian * _weight * _Jacobian.transpose() + _eval);
    Eigen::MatrixXd ___Jacobian = _weight * _Jacobian.transpose() * __Jacobian.inverse();

    Eigen::MatrixXd _delta_angle = ___Jacobian * _err;

    for (int id = 0; id < _idx.size(); id++)
    {
      int _joint_num = _idx[id];
      op3_link_data[_joint_num]->joint_angle += _delta_angle.coeff(id);
    }

    ForwardKinematics(0);
  }

  for (int id = 0; id < _idx.size(); id++)
  {
    int _joint_num = _idx[id];

    if (op3_link_data[_joint_num]->joint_angle >= op3_link_data[_joint_num]->joint_limit_max)
    {
      limit_success = false;
      break;
    }
    else if (op3_link_data[_joint_num]->joint_angle <= op3_link_data[_joint_num]->joint_limit_min)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool OP3KinematicsDynamics::InverseKinematicsforLeg(double *out, double x, double y, double z, double roll,
                                                    double pitch, double yaw)
{
  //Eigen::MatrixXd target_transform;
  Eigen::Matrix4d Tad, Tda, Tcd, Tdc, Tac;
  Eigen::Vector3d vec;

  bool invertible;
  double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
  double THIGH_LENGTH = thigh_length_m;  //std::fabs(robotis_joint[id_r_leg_start + 2*3]->relative_position.coeff(2,0));
  double CALF_LENGTH = calf_length_m;   //std::fabs(robotis_joint[id_r_leg_start + 2*4]->relative_position.coeff(2,0));
  double ANKLE_LENGTH = ankle_length_m;  //std::fabs(robotis_joint[id_r_leg_end]->relative_position(2,0));

  //target_transform = transformationXYZRPY(x, y, z, roll, pitch, yaw);

//
//	Tad << target_transform.coeff(0,0),  target_transform.coeff(0,1),  target_transform.coeff(0,2),  target_transform.coeff(0,3),
//		   target_transform.coeff(1,0),  target_transform.coeff(1,1),  target_transform.coeff(1,2),  target_transform.coeff(1,3),
//		   target_transform.coeff(2,0),  target_transform.coeff(2,1),  target_transform.coeff(2,2),  target_transform.coeff(2,3),
//		   target_transform.coeff(3,0),  target_transform.coeff(3,1),  target_transform.coeff(3,2),  target_transform.coeff(3,3);

  Tad = transformationXYZRPY(x, y, z, roll, pitch, yaw);

  vec.coeffRef(0) = Tad.coeff(0, 3) + Tad.coeff(0, 2) * ANKLE_LENGTH;
  vec.coeffRef(1) = Tad.coeff(1, 3) + Tad.coeff(1, 2) * ANKLE_LENGTH;
  vec.coeffRef(2) = Tad.coeff(2, 3) + Tad.coeff(2, 2) * ANKLE_LENGTH;

  // Get Knee
  _Rac = vec.norm();
  _Acos = acos(
      (_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2.0 * THIGH_LENGTH * CALF_LENGTH));
  if (std::isnan(_Acos) == 1)
    return false;
  *(out + 3) = _Acos;

  // Get Ankle Roll
  Tad.computeInverseWithCheck(Tda, invertible);
  if (invertible == false)
    return false;

  _k = sqrt(Tda.coeff(1, 3) * Tda.coeff(1, 3) + Tda.coeff(2, 3) * Tda.coeff(2, 3));
  _l = sqrt(Tda.coeff(1, 3) * Tda.coeff(1, 3) + (Tda.coeff(2, 3) - ANKLE_LENGTH) * (Tda.coeff(2, 3) - ANKLE_LENGTH));
  _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2.0 * _l * ANKLE_LENGTH);

  if (_m > 1.0)
    _m = 1.0;
  else if (_m < -1.0)
    _m = -1.0;
  _Acos = acos(_m);

  if (std::isnan(_Acos) == 1)
    return false;

  if (Tda.coeff(1, 3) < 0.0)
    *(out + 5) = -_Acos;
  else
    *(out + 5) = _Acos;

  // Get Hip Yaw
  Tcd = transformationXYZRPY(0, 0, -ANKLE_LENGTH, *(out + 5), 0, 0);
  Tcd.computeInverseWithCheck(Tdc, invertible);
  if (invertible == false)
    return false;

  Tac = Tad * Tdc;
  _Atan = atan2(-Tac.coeff(0, 1), Tac.coeff(1, 1));
  if (std::isinf(_Atan) != 0)
    return false;
  *(out) = _Atan;

  // Get Hip Roll
  _Atan = atan2(Tac.coeff(2, 1), -Tac.coeff(0, 1) * sin(*(out)) + Tac.coeff(1, 1) * cos(*(out)));
  if (std::isinf(_Atan) != 0)
    return false;
  *(out + 1) = _Atan;

  // Get Hip Pitch and Ankle Pitch
  _Atan = atan2(Tac.coeff(0, 2) * cos(*(out)) + Tac.coeff(1, 2) * sin(*(out)),
                Tac.coeff(0, 0) * cos(*(out)) + Tac.coeff(1, 0) * sin(*(out)));
  if (std::isinf(_Atan) == 1)
    return false;
  _theta = _Atan;
  _k = sin(*(out + 3)) * CALF_LENGTH;
  _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
  _m = cos(*(out)) * vec.coeff(0) + sin(*(out)) * vec.coeff(1);
  _n = cos(*(out + 1)) * vec.coeff(2) + sin(*(out)) * sin(*(out + 1)) * vec.coeff(0)
      - cos(*(out)) * sin(*(out + 1)) * vec.coeff(1);
  _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
  _c = (_n - _k * _s) / _l;
  _Atan = atan2(_s, _c);
  if (std::isinf(_Atan) == 1)
    return false;
  *(out + 2) = _Atan;
  *(out + 4) = _theta - *(out + 3) - *(out + 2);

  return true;

}

bool OP3KinematicsDynamics::InverseKinematicsforRightLeg(double *out, double x, double y, double z, double roll,
                                                         double pitch, double yaw)
{
  if (InverseKinematicsforLeg(out, x, y, z, roll, pitch, yaw) == true)
  {

    *(out + 0) = out[0] * (op3_link_data[id_r_leg_start + 2 * 0]->joint_axis.coeff(2, 0));
    *(out + 1) = out[1] * (op3_link_data[id_r_leg_start + 2 * 1]->joint_axis.coeff(0, 0));
    *(out + 2) = out[2] * (op3_link_data[id_r_leg_start + 2 * 2]->joint_axis.coeff(1, 0));
    *(out + 3) = out[3] * (op3_link_data[id_r_leg_start + 2 * 3]->joint_axis.coeff(1, 0));
    *(out + 4) = out[4] * (op3_link_data[id_r_leg_start + 2 * 4]->joint_axis.coeff(1, 0));
    *(out + 5) = out[5] * (op3_link_data[id_r_leg_start + 2 * 5]->joint_axis.coeff(0, 0));
    return true;
  }
  else
    return false;
}

bool OP3KinematicsDynamics::InverseKinematicsforLeftLeg(double *out, double x, double y, double z, double roll,
                                                        double pitch, double yaw)
{
  if (InverseKinematicsforLeg(out, x, y, z, roll, pitch, yaw) == true)
  {

    out[0] = out[0] * (op3_link_data[id_l_leg_start + 2 * 0]->joint_axis.coeff(2, 0));
    out[1] = out[1] * (op3_link_data[id_l_leg_start + 2 * 1]->joint_axis.coeff(0, 0));
    out[2] = out[2] * (op3_link_data[id_l_leg_start + 2 * 2]->joint_axis.coeff(1, 0));
    out[3] = out[3] * (op3_link_data[id_l_leg_start + 2 * 3]->joint_axis.coeff(1, 0));
    out[4] = out[4] * (op3_link_data[id_l_leg_start + 2 * 4]->joint_axis.coeff(1, 0));
    out[5] = out[5] * (op3_link_data[id_l_leg_start + 2 * 5]->joint_axis.coeff(0, 0));
    return true;
  }
  else
    return false;
}

}
