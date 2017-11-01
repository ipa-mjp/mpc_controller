
#ifndef MPC_CONTROLLER_UTILITIES_MPC_UTILITIES_H
#define MPC_CONTROLLER_UTILITIES_MPC_UTILITIES_H

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

namespace nmpc
{
	class MpcUtilities
	{

		private:

			ros::NodeHandle nh_;
			ros::Subscriber jointState_sub_;

			ControllerParam controller_param_;

			KDL::Chain chain_;
			JointStates joint_states_;

			tf::TransformListener tf_listener_;

		public:
			MpcUtilities() = default;
			~MpcUtilities() {};

			bool initialize(void);
			void jointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);

			void unitTestFunctionlity(void);

	};

	#endif //MPC_CONTROLLER_UTILITIES_MPC_UTILITIES_H

}  // namespace nmpc


