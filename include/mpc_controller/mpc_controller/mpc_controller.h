
#ifndef MPC_CONTROLLER_MPC_CONTROLLER_MPC_CONTROLLER_H
#define MPC_CONTROLLER_MPC_CONTROLLER_MPC_CONTROLLER_H

#include <iostream>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <urdf/model.h>

//KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

//BOOST
#include <boost/shared_ptr.hpp>

#include <mpc_controller/data_types.h>
#include <mpc_controller/Kinematics/kinematics.h>

namespace nmpc
{
	class MpcController
	{

		private:

			ros::NodeHandle nh_;
			ros::Subscriber jointState_sub_;

			ControllerParam controller_param_;

			//KDL::Chain chain_;
			JointStates joint_states_;

			tf::TransformListener tf_listener_;

			boost::shared_ptr<Kinematics> kinematics_solver_ptr_;

		public:
			MpcController() = default;
			~MpcController() {};

			bool initialize(void);
			void jointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);

			void unitTestFunctionlity(void);
			ControllerParam getControllerParamObject(void)
			{
				return controller_param_;
			}
	};

	#endif //MPC_CONTROLLER_MPC_CONTROLLER_MPC_CONTROLLER_H

}  // namespace nmpc


