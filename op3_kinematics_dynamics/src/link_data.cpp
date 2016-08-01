/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "op3_kinematics_dynamics/link_data.h"


namespace ROBOTIS
{

LinkData::LinkData()
{
    name = "";

    parent = -1;
    sibling = -1;
    child = -1;

    mass = 0.0;

    relative_position = transitionXYZ( 0.0 , 0.0 , 0.0 );
    joint_axis = transitionXYZ( 0.0 , 0.0 , 0.0 );
    center_of_mass = transitionXYZ( 0.0 , 0.0 , 0.0 );
    inertia = inertiaXYZ( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 );

    joint_limit_max = 100.0;
    joint_limit_min = -100.0;

    joint_angle = 0.0;
    joint_velocity = 0.0;
    joint_acceleration = 0.0;

    position = transitionXYZ( 0.0 , 0.0 , 0.0 );
    orientation = rpy2rotation( 0.0 , 0.0 , 0.0 );
    transformation = transformationXYZRPY( 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0);
}

LinkData::~LinkData(){}

}
