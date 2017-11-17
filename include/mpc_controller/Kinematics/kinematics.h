
#ifndef MPC_CONTROLLER_KINEMATICS_H_
#define MPC_CONTROLLER_KINEMATICS_H_

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_kdl/tf2_kdl.h>

//KDL kinematics
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

//C++
#include <iostream>
#include <map>
#include <string>

//ACADO
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_optimal_control.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

#include <mpc_controller/data_types.h>

/**
 * @brief class for computing forward kinematics and inverse kinematics
 */
#define _DEBUG_  false

namespace nmpc
{

	class Kinematics
	{
	private:

		unsigned int segments;
		unsigned int dof;

		std::string chain_base_link;
		std::string chain_tip_link;
		std::string root_frame;

		std::vector<KDL::Vector> jnt_rot_axis;
		std::vector<std::vector<double> > jnt_rot_angle;

		KDL::Chain	kinematic_chain;
		std::vector<KDL::Frame>	frames;
		std::vector<KDL::Joint>	jnts;
		std::vector<KDL::Frame>	jnt_homo_mat;	//homo matrix of each frame with prevoius joint

		std::vector<KDL::Frame> jnt_fk_mat;	//ff_mat
		KDL::Frame fk_mat;
		Eigen::Matrix<double, 6, 7> JacobianMatrix;	//Jacobian Matrix

		//bool _DEBUG_;
		void printDataMemebers(void);

		void createHomoRoatationMatrix(const uint16_t& seg_nr);
		void createRoatationMatrix(const double& angle, const std::vector<unsigned int>& rot_axis, KDL::Frame& lcl_homo_mat);

	public:

		//Kinematics(const std::string rbt_description = "/robot_description", const std::string& chain_base_link="arm_base_link", const std::string& chain_tip_link="arm_7_link", const std::string& root_frame="world");

		Kinematics():
			dof(0),
			segments(0),
			chain_base_link("arm_1_link"),
			chain_tip_link("arm_7_link"),
			root_frame("world")
		{};
		~Kinematics(){};

		void initialize(const std::string rbt_description = "/robot_description", const std::string& chain_base_link="arm_base_link", const std::string& chain_tip_link="arm_7_link", const std::string& root_frame="world");
		void initialize(const KDL::Chain& kinematic_chain, const std::string& chain_base_link, const std::string& chain_tip_link, const std::string& root_frame );
		bool initialize(const ControllerParam& controller_param );

		void forwardKinematics(const KDL::JntArray& jnt_angels);

		void computeJacobian(const KDL::JntArray& jnt_angels);

		void kdl_forwardKinematics(const KDL::JntArray& jnt_angels);

		void kdl_computeJacobian(const KDL::JntArray& jnt_angels);

		//void computeInverseJacobianBySVD();

		//get functions
		std::string getChainBaseLink(void);
		void getChainBaseLink(std::string& base_link);

		std::string getChainTipLink(void);
		void getChainTipLink(std::string& tip_link);

		std::string getChainRootLink(void);
		void getChainRootLink(std::string& root_frame);

		uint16_t getNumberOfJnts(void);
		void getNumberOfJnts(uint16_t& nr_jnts);

		uint16_t getNumberOfSegments(void);
		void getNumberOfSegments(uint16_t& nr_segments);

		std::vector<KDL::Joint> getJntsInfo(void);
		void getJntsInfo(std::vector<KDL::Joint>& jnts);

		void getForwardKinematics(KDL::Frame& fk_mat);
		void getForwardKinematics(Eigen::MatrixXd& fk_mat);
		KDL::Frame getForwardKinematics(void);
		//Eigen::MatrixXd getForwardKinematics(void);

		void getJacobian(const KDL::JntArray& jnt_angles, Eigen::MatrixXd& j_mat);
		Eigen::MatrixXd getJacobian(const KDL::JntArray& jnt_angles);

		//set functions

	};

	#endif	//MPC_CONTROLLER_KINEMATICS_H_
}


