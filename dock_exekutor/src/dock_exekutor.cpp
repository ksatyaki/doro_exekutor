/*
 * dock_exekutor.cpp
 *
 *  Created on: Sep 3, 2014
 *      Author: ace
 */

#include <exekutor/dock_exekutor.h>

namespace exekutor {

DockExekutor::DockExekutor(std::string robot_name, std::string action_name):
				ActionExekutor(robot_name, action_name),
				move_to_simple_client_("move_to_simple", true)
{

}

DockExekutor::~DockExekutor()
{

}

void DockExekutor::actionThread()
{
	ROS_INFO("Dock/Undock-Exekutor has started.");
	PeisTuple paramTuple = getParamTuple();

	std::vector <std::string> paramStrings = extractParamStrings(paramTuple.data);

	if(paramStrings.size() > 1)
	{
		ROS_INFO("Action failed. Reason: Too many parameters.");
		setState(FAILED);
		return;
	}

	for(int i = 0; i<paramStrings[0].size(); i++)
		paramStrings[0][i] = tolower(paramStrings[0][i]);

	if(paramStrings[0].compare("undock") == 0 || paramStrings[0].compare("UNDOCK") == 0)
	{
		ROS_INFO("Undocking...");
		undockAction();
	}
	else
		dockAction(paramStrings[0]);
}

void DockExekutor::undockAction()
{
	if(last_operation == UNDOCK)
	{
		ROS_INFO("Action failed. Last action was an undock. Undock can only be performed after a dock.");
		setState(FAILED);
		return;
	}

	simple_service::MoveToSimpleGoal _goal;

	_goal.driving_direction = simple_service::MoveToSimpleGoal::REVERSE;
	_goal.goal_pose.pose.position.x = undock_goal.pose.position.x;
	_goal.goal_pose.pose.position.y = undock_goal.pose.position.y;
	_goal.goal_pose.pose.orientation.z = 0.0;
	_goal.goal_pose.pose.orientation.w = 1.0;

	move_to_simple_client_.waitForServer();
	ROS_INFO("Undocking robot...");
	move_to_simple_client_.sendGoal(_goal);
	move_to_simple_client_.waitForResult();

	if(move_to_simple_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Undocking sucked while moving back. Sorry.");
		setState(FAILED);
		return;
	}

	_goal.goal_pose.pose.position.x = 0.0;
	_goal.goal_pose.pose.position.y = 0.0;
	_goal.goal_pose.pose.orientation.z = undock_goal.pose.orientation.z;
	_goal.goal_pose.pose.orientation.w = undock_goal.pose.orientation.w;
	move_to_simple_client_.sendGoal(_goal);
	ROS_INFO("Final rotate...");
	move_to_simple_client_.waitForResult();

	ROS_INFO("Done.");
	last_operation = UNDOCK;
	undock_goal.pose.position.x = 0.0;
	undock_goal.pose.position.y = 0.0;
	undock_goal.pose.orientation.z = 0.0;
	undock_goal.pose.orientation.w = 1.0;
	setState(COMPLETED);
	return;
}

void DockExekutor::dockAction(const std::string& object_name)
{

	geometry_msgs::PointStamped obj_position = cam_interface::getObjectPositionFromCAM(object_name.c_str(), this->tf_listener_);

	if(obj_position.header.frame_id.compare("para-universe") == 0)
	{
		ROS_INFO("There was a problem retriving the position of object from CAM. Action FAILED.");
		setState(FAILED);
		return;
	}

	else if(obj_position.header.frame_id.compare("cocked-up") == 0)
	{
		ROS_INFO("Transform Failure. Action FAILED.");
		setState(FAILED);
		return;
	}

	double reqd_dist = sqrt((obj_position.point.x*obj_position.point.x) + (obj_position.point.y*obj_position.point.y)) - 0.40;
	double reqd_pan = atan2(obj_position.point.y, obj_position.point.x) + 0.1;

	simple_service::MoveToSimpleGoal _goal;

	// Turn first.
	_goal.goal_pose.pose.orientation.w = cos(reqd_pan/2);
	_goal.goal_pose.pose.orientation.z = sin(reqd_pan/2);

	move_to_simple_client_.waitForServer();
	ROS_INFO("Rotating first.");
	move_to_simple_client_.sendGoal(_goal);
	move_to_simple_client_.waitForResult();

	if(move_to_simple_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Docking sucked while trying to rotate the robot. Sorry.");
		setState(FAILED);
		return;
	}

	if(reqd_dist < 0.0)
	{
		ROS_INFO("Hurray! We docked the robot near the object.");
		setState(COMPLETED);
		last_operation = DOCK;
		undock_goal.pose.position.x = reqd_dist;
		undock_goal.pose.position.y = 0.0;
		undock_goal.pose.orientation.z = sin(-1*reqd_pan/2);
		undock_goal.pose.orientation.w = cos(-1*reqd_pan/2);

		return;
	}
	_goal.goal_pose.pose.position.x = reqd_dist;
	_goal.goal_pose.pose.position.y = 0.0;

	_goal.goal_pose.pose.orientation.w = 1.0;
	_goal.goal_pose.pose.orientation.z = 0.0;

	ROS_INFO("Moving closer now.");
	move_to_simple_client_.sendGoal(_goal);
	move_to_simple_client_.waitForResult();

	if(move_to_simple_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Docking sucked when trying to get closer. Sorry.");
		setState(FAILED);
		return;
	}
	else
	{
		ROS_INFO("Hurray! We docked the robot near the object.");
		setState(COMPLETED);
		last_operation = DOCK;
		undock_goal.pose.position.x = -1 * reqd_dist;
		undock_goal.pose.position.y = 0.0;
		undock_goal.pose.orientation.z = sin(-1*reqd_pan/2);
		undock_goal.pose.orientation.w = cos(-1*reqd_pan/2);

		return;
	}
}


} /* namespace exekutor */
