/*
 * moveit_hand_exekutor.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: ace
 */

#include "exekutor/moveit_hand_exekutor.h"

namespace exekutor {

MoveitHandExekutor::MoveitHandExekutor(std::string robot_name, std::string action_name):
		ActionExekutor (robot_name, action_name),
		pam_client_ ("plan_and_move_arm", true)
{

}

MoveitHandExekutor::~MoveitHandExekutor()
{

}

void MoveitHandExekutor::actionThread()
{
	ROS_INFO("MoveitHandExekutor has started.");

	PeisTuple paramTuple = getParamTuple();

	std::vector<double> values = extractParams(paramTuple.data);

	if(values.size() != 7)
	{
		ROS_INFO("The values should be exactly 7 numbers. Action failed!");
		setState(FAILED);
		return;
	}

	doro_manipulation::PlanAndMoveArmGoal _goal;
	_goal.goal_type = "pose";
	_goal.target_pose.header.frame_id = "base_link";
	_goal.target_pose.pose.position.x = values[0];
	_goal.target_pose.pose.position.y = values[1];
	_goal.target_pose.pose.position.z = values[2];
	_goal.target_pose.pose.orientation.x = values[3];
	_goal.target_pose.pose.orientation.y = values[4];
	_goal.target_pose.pose.orientation.z = values[5];
	_goal.target_pose.pose.orientation.w = values[6];

	pam_client_.waitForServer();
	ROS_INFO("Calling doro_manipulation...");
	pam_client_.sendGoal(_goal);
	pam_client_.waitForResult();

	if(pam_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Move succeeded.");
		setState(COMPLETED);
		return;
	}
	else
	{
		ROS_INFO("Move failed.");
		setState(FAILED);
		return;
	}
}

} /* namespace exekutor */
