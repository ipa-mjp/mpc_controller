
#include <ros/ros.h>
#include <mpc_controller/mpc_solver/mpc_solver_ACADO.h>
//#include <mpc_controller/inverse_jacobian_calculation/inverse_jacobian_calculation.h>
#include <mpc_controller/mpc_controller/mpc_controller.h>


#define _DEBUG_  false
using namespace nmpc;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematic_test");
	ros::NodeHandle node_handler;

	if (node_handler.hasParam("/robot_description"))
	{
		MpcController mpc_util;
		mpc_util.initialize();
		mpc_util.unitTestFunctionlity();
		ros::Duration(5.0).sleep();

	}
return 0;
}
