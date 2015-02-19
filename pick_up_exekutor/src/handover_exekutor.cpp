/*
 * handover_exekutor.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: ace
 */

#include <exekutor/handover_exekutor.h>

namespace exekutor {

HandoverExekutor::HandoverExekutor(std::string robot_name, std::string action_name) :
		ActionExekutor (robot_name, action_name),
		pam_client_ ("plan_and_move_arm", true)
{
	release_msg.data.push_back(0.0);
	release_msg.data.push_back(0.0);
	release_msg.data.push_back(0.0);

	fingers_pub_ = nh_.advertise <std_msgs::Float64MultiArray> ("jaco/finger_command", 1);

}

void HandoverExekutor::jacoJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	prev_joint_3_value = current_joint_3_value;
	current_joint_3_value = boost::shared_ptr<double> (new double(msg->effort[2]));
}



void HandoverExekutor::actionThread()
{
	ROS_INFO("Handover exekutor has started");

	std::string param = getParamTuple().data;

	if(param.compare("EXTEND") == 0 || param.compare("extend") == 0)
	{
		// Step 1: Extend.
		pam_client_.waitForServer();
		doro_manipulation::PlanAndMoveArmGoal pam_goal;
		pam_goal.goal_type = "handover";
		ROS_INFO("Handing over...");
		pam_client_.sendGoal(pam_goal);
		pam_client_.waitForResult();
		if(pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			setState(FAILED);
			ROS_INFO("Moving to handover pose failed.");
			return;
		}
	}

	else
	{
		// Step 2: Wait for joint_states to set the value of hand_over_requested
		ros::Subscriber joint_states_sub = nh_.subscribe("jaco/joint_states", 1, &HandoverExekutor::jacoJointStatesCallback, this);

		while(ros::ok())
		{
			ros::spinOnce();
			if(!prev_joint_3_value || !current_joint_3_value)
			{
				ROS_INFO("Wait for meaningful messages.");
			}
			else
			{
				if(fabs(*prev_joint_3_value - *current_joint_3_value) > 0.025)
				{
					ROS_INFO("Releasing grasp.");
					releaseHold();
					prev_joint_3_value.reset();
					current_joint_3_value.reset();
					break;
				}
				else
					ROS_INFO("Difference is: %lf", fabs(*prev_joint_3_value - *current_joint_3_value));
			}
			usleep(200000);
		}

		joint_states_sub.shutdown();

		// Step 3: Retract.
		pam_client_.waitForServer();
		doro_manipulation::PlanAndMoveArmGoal pam_goal;
		pam_goal.goal_type = "home";
		ROS_INFO("Retracting...");
		pam_client_.sendGoal(pam_goal);
		pam_client_.waitForResult();
		if(pam_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			setState(FAILED);
			ROS_INFO("Home-ing the arm failed.");
			return;
		}

	}

	setState(COMPLETED);
	ROS_INFO("Action succeeded.");
}



HandoverExekutor::~HandoverExekutor()
{

}

} /* namespace exekutor */
