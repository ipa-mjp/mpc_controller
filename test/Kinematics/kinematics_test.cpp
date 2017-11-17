
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mpc_controller/mpc_solver/mpc_solver_ACADO.h>
//#include <mpc_controller/inverse_jacobian_calculation/inverse_jacobian_calculation.h>
#include <mpc_controller/mpc_controller/mpc_controller.h>

using namespace nmpc;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinematic_test");
	ros::NodeHandle node_handler;
	//ros::AsyncSpinner spinner(4); // Use 4 threads, be aware to use it
	//spinner.start();

	if (node_handler.hasParam("/robot_description"))
	{
		/*
		MpcController mpc_util;
		mpc_util.initialize();
		//ros::Duration(5.0).sleep();
		mpc_util.unitTestFunctionlity();
		//ros::waitForShutdown();
		*/

		Kinematics kin_solver;
		kin_solver.initialize();

		KDL::JntArray jnt_angles = KDL::JntArray(7);
		jnt_angles(0) = 1.57;	jnt_angles(1) = 1.57;	jnt_angles(2) = 1.57;	jnt_angles(3) = 1.57;
		//kin_solver.kdl_computeJacobian(jnt_angles);
		std::cout << kin_solver.getJacobian(jnt_angles) << std::endl;


		/*
		ModelPredictiveControlACADO acado_solver_;
		acado_solver_.initializeClassMembers(J);
		//acado_solver_.solve();
		acado_solver_.hard_coded_solve();
		*/
	}
return 0;
}
