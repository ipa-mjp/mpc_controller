
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

//--------------------------------------------------------------------------

struct LimiterParam
{
	LimiterParam():
		limits_tolerance(1.0)
		{;}

	double limits_tolerance;

	std::vector<double> pos_min_limit_;
	std::vector<double> pos_max_limit_;
	std::vector<double> vel_limit_;
	//std::vector<double> vel_max_limit_;
};


//---------------------------------------------------------------------------------------------------------------------------
struct ControllerParam
{

	ControllerParam():
		dof(0),
		chain_base_link("arm_base_link"),
		chain_tip_link("arm_7_link"),
		root_frame("arm_base_link")
		{
			;
		}

	uint8_t dof;
	std::string chain_base_link;
	std::string chain_tip_link;
	std::string root_frame;

	std::vector<std::string> frame_names;
	std::vector<std::string> jnts_name;

	LimiterParam lmt_param_;

}; 

//----------------------------------------------------------------------------------------------------------------------------

typedef Eigen::Matrix<double, 6, Eigen::Dynamic> 	Matrix6Xd_t		;
typedef Eigen::Matrix<double, 6, 7> 				JacobianMatrix	;	//Jacobian matrix for care-o-bot with 7 dof
typedef Eigen::Matrix<double, 6, 1> 				Cart6dVector	;
typedef Eigen::Matrix<double, 3, 1> 				Cart3dVector	;

#endif //MPC_CONTROLLER_DATA_TYPES_H
