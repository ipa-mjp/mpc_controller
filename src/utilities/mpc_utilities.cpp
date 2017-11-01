
#include <mpc_controller/utilities/mpc_utilities.h>

using namespace nmpc;

bool MpcUtilities::initialize(void)
{

	//Joint names
	if (!nh_.getParam ("joint_names", controller_param_.jnts_name) )
	{
		ROS_WARN(" Parameter 'joint names' not set on %s node " , ros::this_node::getName().c_str());
	}

	//dof
	controller_param_.dof = controller_param_.jnts_name.size();

	//Chain_base and chain tip links, root frame
	if (!nh_.getParam ("chain_base_link", controller_param_.chain_base_link) )
	{
		ROS_WARN(" Parameter 'chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh_.getParam ("chain_tip_link", controller_param_.chain_tip_link) )
	{
		ROS_WARN(" Parameter 'chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh_.getParam ("root_frame", controller_param_.root_frame) )
	{
		ROS_WARN(" Parameter 'root_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	// generate KDL chain and urdf model by parse robot description
	KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromParam("/robot_description", kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    kdl_tree.getChain( controller_param_.chain_base_link, controller_param_.chain_tip_link, this->chain_ );
    if (this->chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }

    for (uint16_t i = 0; i < controller_param_.dof ; ++i)
    {
    	controller_param_.lmt_param_.pos_min_limit_.push_back(model.getJoint
    			(controller_param_.jnts_name[i])->limits->lower);
    	controller_param_.lmt_param_.pos_max_limit_.push_back(model.getJoint
    			(controller_param_.jnts_name[i])->limits->upper);
    	controller_param_.lmt_param_.vel_limit_.push_back(model.getJoint
    			(controller_param_.jnts_name[i])->limits->velocity);

    }


    //Set frame Name
    controller_param_.frame_names.clear();
    for (uint16_t i = 0; i < this->chain_.getNrOfSegments(); ++i)
    {
    	controller_param_.frame_names.push_back(this->chain_.getSegment(i).getName());
    }

	//initialize current and last joint values and velocities
	this->joint_states_.current_q_ 		= KDL::JntArray( this->chain_.getNrOfJoints() );
	this->joint_states_.current_q_dot_ 	= KDL::JntArray( this->chain_.getNrOfJoints() );
	this->joint_states_.last_q_ 			= KDL::JntArray( this->chain_.getNrOfJoints() );
	this->joint_states_.last_q_dot_ 		= KDL::JntArray( this->chain_.getNrOfJoints() );

	//initialize ROS interfaces
	this->jointState_sub_ = nh_.subscribe("/joint_states", 1, &MpcUtilities::jointStateCallBack, this);

	//Info message for initialized utilities class
	std::cout<<"\033[36;1m" <<"MpcUtilities... initialization done!" <<"\033[36;0m"<<std::endl;

	return true;
}

void MpcUtilities::jointStateCallBack(const sensor_msgs::JointState::ConstPtr&  msg)
{

	std::cout<<"\033[94m" << "\033[1m" <<" MpcUtilities... jointStateCallBack "<<"\033[0;0m"<<std::endl;

	ros::Duration(0.1).sleep();

    KDL::JntArray q_lcl = this->joint_states_.current_q_;
    KDL::JntArray q_dot_lcl = this->joint_states_.current_q_dot_;
    int count = 0;	//valid joints

    for (uint16_t i = 0; i < controller_param_.dof ; ++i)
    {
    	for (uint16_t j = 0; j < msg->name.size() ; ++j)
    	{
    		//compare msg joint with kinematics chain joint
    		if (std::strcmp( msg->name[j].c_str(), controller_param_.jnts_name[i].c_str() ) == 0)
    		{
    			q_lcl(i) = msg->position[j];
    			q_dot_lcl(i) = msg->velocity[j];
    			count++;
    			break;
    		}

    	}
    }

    //update position and velocity
    if ( count == controller_param_.jnts_name.size())
    {
    	this->joint_states_.last_q_ = this->joint_states_.current_q_;
    	this->joint_states_.last_q_dot_ = this->joint_states_.current_q_dot_;
    	this->joint_states_.current_q_ = q_lcl;
    	this->joint_states_.current_q_dot_ = q_dot_lcl;
    }
/*
	//-----------------------------------------------------------
	std::cout<<"\033[0;31m"	<< "last_q_: \n" 		<< this->joint_states_.last_q_.data 		<<"\033[0;0m"<<std::endl;
	std::cout<<"\033[0;33m"	<< "last_q_dot_: \n" 	<< this->joint_states_.last_q_dot_.data 	<<"\033[0;0m"<<std::endl;
	std::cout<<"\033[0;32m"	<< "current_q_: \n" 	<< this->joint_states_.current_q_.data 		<<"\033[0;0m"<<std::endl;
	std::cout<<"\033[94m"	<< "current_q_dot_: \n" << this->joint_states_.current_q_dot_.data 	<<"\033[94;0m"<<std::endl;
*/

}

void MpcUtilities::unitTestFunctionlity(void)
{

	bool init_success = this->initialize();

	if (init_success)
	{
		//-----------------------------------------------------------
		std::cout<< "\033[95m"	<< "Chain_base_link_:"	<< this->controller_param_.chain_base_link				<<"\033[0;0m"<<std::endl;
		std::cout<< "\033[95m"	<< "Chain_tip_link_ :" 	<< this->controller_param_.chain_tip_link				<<"\033[0;0m"<<std::endl;
		std::cout<< "\033[95m"	<< "Root_frame_	:" 		<< this->controller_param_.root_frame					<<"\033[0;0m"<<std::endl;
		std::cout<< "\033[95m"	<< "DOF  		:" 		<< this->controller_param_.dof							<<"\033[0;0m"<<std::endl;
		std::cout<< "\033[95m"	<< "Limit_tolerance :" 	<< this->controller_param_.lmt_param_.limits_tolerance	<<"\033[0;0m"<<std::endl;

		//-----------------------------------------------------------
		std::cout<< "\033[95m"	<< "Joint_names :[ ";
		for (uint16_t i = 0; i < this->controller_param_.dof; ++i)
		{
			std::cout<< "\033[95m" 	<< this->controller_param_.jnts_name[i] << " , ";
		}
		std::cout<< " ]"<<"\033[0;0m"<<std::endl;

		//-----------------------------------------------------------
		std::cout<< "\033[95m"	<< "Joint_pose_limits :[ ";
		for (uint16_t i = 0; i < this->controller_param_.dof; ++i)
		{
			std::cout<< "\033[95m" <<" [ "<< this->controller_param_.lmt_param_.pos_min_limit_[i]<<" , "
			                       	<<this->controller_param_.lmt_param_.pos_max_limit_[i] << " ] "<< " , ";
		}
		std::cout<< " ]"<<"\033[0;0m"<<std::endl;

		//-----------------------------------------------------------
		std::cout<< "\033[95m"	<< "Joint_velocity_limits :[ ";
		for (uint16_t i = 0; i < this->controller_param_.dof; ++i)
		{
			std::cout<< "\033[95m" << this->controller_param_.lmt_param_.vel_limit_[i] << " , ";
		}
		std::cout<< " ]"<<"\033[0;0m"<<std::endl;

		//-----------------------------------------------------------
		std::cout<<"\033[0;31m"	<< "last_q_: \n" 		<< this->joint_states_.last_q_.data 		<<"\033[0;0m"<<std::endl;
		std::cout<<"\033[0;33m"	<< "last_q_dot_: \n" 	<< this->joint_states_.last_q_dot_.data 	<<"\033[0;0m"<<std::endl;
		std::cout<<"\033[0;32m"	<< "current_q_: \n" 	<< this->joint_states_.current_q_.data 		<<"\033[0;0m"<<std::endl;
		std::cout<<"\033[94m"	<< "current_q_dot_: \n" << this->joint_states_.current_q_dot_.data 	<<"\033[94;0m"<<std::endl;
	}
}


