#ifndef OP3_KINEMATICS_DYNAMICS_H_
#define OP3_KINEMATICS_DYNAMICS_H_


#include <vector>

#include "OP3KinematicsDynamicsDefine.h"
#include "LinkData.h"

namespace ROBOTIS
{

enum TREE_SELECT {
    MANIPULATION,
    WALKING,
    WHOLE_BODY
};

class OP3KinematicsDynamics
{

public:

	LinkData *op3_link_data [ ALL_JOINT_ID + 1 ];

	double thigh_length_m;
	double calf_length_m;
	double ankle_length_m;
	double leg_side_offset_m;

	OP3KinematicsDynamics();
    ~OP3KinematicsDynamics();
    OP3KinematicsDynamics(TREE_SELECT tree);

    std::vector<int> findRoute( int to );
    std::vector<int> findRoute( int from , int to );

    double TotalMass( int joint_ID );
    Eigen::MatrixXd CalcMC( int joint_ID );
    Eigen::MatrixXd CalcCOM( Eigen::MatrixXd MC );

    void ForwardKinematics( int joint_ID );

    Eigen::MatrixXd CalcJacobian( std::vector<int> idx );
    Eigen::MatrixXd CalcJacobianCOM( std::vector<int> idx );
    Eigen::MatrixXd CalcVWerr( Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position, Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation );

    bool InverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation , int max_iter, double ik_err);
    bool InverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err );

    // with weight
    bool InverseKinematics( int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight );
    bool InverseKinematics( int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation, int max_iter, double ik_err , Eigen::MatrixXd weight );

    bool InverseKinematicsforLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
    bool InverseKinematicsforRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
    bool InverseKinematicsforLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw);
};

}

#endif /* OP3_KINEMATICS_DYNAMICS_H_ */
