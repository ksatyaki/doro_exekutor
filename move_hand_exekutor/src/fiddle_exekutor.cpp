/*
 * fiddle_exekutor.cpp
 *
 *  Created on: Jul 10, 2014
 *      Author: ace
 */

#include <exekutor/fiddle_exekutor.h>

namespace exekutor {

FiddleExekutor::FiddleExekutor(std::string robot_name, std::string action_name) :
		ActionExekutor(robot_name, action_name),
		finger_action_client_("jaco/finger_action", true)
{

}

FiddleExekutor::~FiddleExekutor()
{
}

void FiddleExekutor::actionThread()
{
	ROS_INFO("FiddleExekutor has started.");

	setState(RUNNING);

	PeisTuple paramTuple = getParamTuple();

	std::vector <std::string> params = extractParamStrings(paramTuple.data);

	std::string type_str = params[0];
	std::vector<double> the_values;

	if(params.size() > 1)
		the_values = extractParams(params[1].c_str());

	int num_arg = the_values.size();
	// The goal variable.
	jaco::FingerMovementGoal our_goal;

	if(type_str.compare("open") == 0)
	{
		our_goal.task = jaco::FingerMovementGoal::OPEN;
	}
	else if(type_str.compare("close") == 0)
	{
		our_goal.task = jaco::FingerMovementGoal::CLOSE;
	}
	else if(type_str.compare("absolute") == 0 && (num_arg == 3 || num_arg == 1) )
	{
		our_goal.task = jaco::FingerMovementGoal::ABS;
		if(num_arg == 3)
		{
			our_goal.abs_position.push_back(the_values[0]);
			our_goal.abs_position.push_back(the_values[1]);
			our_goal.abs_position.push_back(the_values[2]);
		}
		else
		{
			our_goal.abs_position.push_back(the_values[0]);
			our_goal.abs_position.push_back(the_values[0]);
			our_goal.abs_position.push_back(the_values[0]);
		}
	}
	else
	{
		ROS_INFO("Wrong format. The format should be {open/close/absoulte}");
		ROS_INFO("followed by one or three floating point values if the first argument is \'absolute\'.");
		ROS_INFO("Eg. 'open' or 'close' or 'absolute 1.0 0.5 1.3");

		setState(FAILED);
		return;
	}

	finger_action_client_.waitForServer();
	ROS_INFO("Server connected. Sending goal...");
	finger_action_client_.sendGoal(our_goal);

	finger_action_client_.waitForResult();

	if(finger_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Successfully executed the action on the robot.");
		setState(COMPLETED);
	}
	else
	{
		ROS_INFO("\n\nAttempted! But, something cocked-up and we failed!");
		setState(FAILED);
	}
}

} /* namespace exekutor */
