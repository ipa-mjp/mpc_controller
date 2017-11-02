
#ifndef MPC_CONTROLLER_DATA_TYPES_H
#define MPC_CONTROLLER_DATA_TYPES_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <acado/acado_toolkit.hpp>

//TYPE OF JOINT USED FOR COMPUTATION OF FORWARD KINEMATICS
enum JointType
{
	REVOLUTE 	= 0,
	PRESMATIC 	= 8,
	FIXED		= 5
};


/*
//TYPE OF JOINT USED FOR COMPUTATION OF FORWARD KINEMATICS
struct JointType
{
	static constexpr uint16_t REVOLUTE = 0;
	static constexpr uint16_t PRESMATIC= 8;
	static constexpr uint16_t FIXED	   = 5;
};
*/

//------------------------------------------------------------------------

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
	KDL::Chain chain_;

}; 

//----------------------------------------------------------------------------------------------------------------------------

struct AcadoConfigParam
{
	uint16_t initial_max_num_iterations_;

	uint16_t discretization_steps_;
	uint16_t min_discretization_steps_;
	uint16_t max_discretization_steps_;

	double initial_kkt_tolerance_;
	double max_time_horizon_;

	//hard constraints
	double x_at_start_;
	double x_at_end_;
	double u_at_start_;
	double u_at_end_;

	//trajectories: state, controls, parameters
	ACADO::VariablesGrid state_trajectory_;
	ACADO::VariablesGrid control_trajectory_;
	ACADO::VariablesGrid parameter_trajectory_;

	//initialization for states, controls and parameters
	ACADO::VariablesGrid state_initialize_;
	ACADO::VariablesGrid control_initialize_;
	ACADO::VariablesGrid parameter_initialize_;

	//output for states, controls and parameters
	ACADO::VariablesGrid state_output_;
	ACADO::VariablesGrid control_output_;
	ACADO::VariablesGrid parameter_output_;

};


//---------------------------------------------------------------------------
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> 	Matrix6Xd_t		;
typedef Eigen::MatrixXd								JacobianMatrix	;
//typedef Eigen::Matrix<double, 6, 7> 				JacobianMatrix	;	//Jacobian matrix for care-o-bot with 7 dof
typedef Eigen::Matrix<double, 6, 1> 				Cart6dVector	;
typedef Eigen::Matrix<double, 3, 1> 				Cart3dVector	;

#endif //MPC_CONTROLLER_DATA_TYPES_H
