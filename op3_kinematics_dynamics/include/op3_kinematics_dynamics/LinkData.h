#ifndef LINK_DATA_H_
#define LINK_DATA_H_

#include "robotis_math/RobotisMath.h"

namespace ROBOTIS
{

class LinkData
{
public:
	LinkData();
    ~LinkData();

    std::string name;

    int parent;
    int sibling;
    int child;

    double mass;

    Eigen::MatrixXd relative_position;
    Eigen::MatrixXd joint_axis;
    Eigen::MatrixXd center_of_mass;
    Eigen::MatrixXd inertia;

    double joint_limit_max;
    double joint_limit_min;

    double joint_angle;
    double joint_velocity;
    double joint_acceleration;

    Eigen::MatrixXd position;
    Eigen::MatrixXd orientation;
    Eigen::MatrixXd transformation;
};

}

#endif /* MANIPULATION_MODULE_ROBOTISLINK_H_ */
