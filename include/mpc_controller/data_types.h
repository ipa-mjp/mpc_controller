
#ifndef MPC_CONTROLLER_DATA_TYPES_H
#define MPC_CONTROLLER_DATA_TYPES_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>

//Also known as DifferntialState in ACADO Toolkit
struct JointStates
{
    KDL::JntArray current_q_;
    KDL::JntArray last_q_;
    KDL::JntArray current_q_dot_;
    KDL::JntArray last_q_dot_;
};


typedef Eigen::Matrix<double, 6, Eigen::Dynamic> 	Matrix6Xd_t		;
typedef Eigen::Matrix<double, 6, 7> 				JacobianMatrix	;	//Jacobian matrix for care-o-bot with 7 dof
typedef Eigen::Matrix<double, 6, 1> 				Cart6dVector	;
typedef Eigen::Matrix<double, 3, 1> 				Cart3dVector	;

#endif //MPC_CONTROLLER_DATA_TYPES_H
