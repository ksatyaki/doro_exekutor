/*
 * move_hand_exekutor.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: ace
 */

#include <exekutor/move_hand_exekutor.h>

namespace exekutor {

void* spin_once(void *dummy)
{
	ros::spin();
	return NULL;
}

MoveHandExekutor::MoveHandExekutor(std::string robot_name, std::string action_name) :
		ActionExekutor(robot_name, action_name),
		joints_client_ ("jaco/jaco_arm_controller/joint_trajectory_action", true),
		cartesian_client_ ("jaco/cartesian_action", true)
{

}

MoveHandExekutor::~MoveHandExekutor()
{
}

void MoveHandExekutor::jointStatesCallback(const sensor_msgs::JointStateConstPtr& _msg)
{
	current_joint_state_ = *_msg;
	ROS_INFO("Fetch joint states");
}

void MoveHandExekutor::actionThread()
{
	ROS_INFO("MoveHandExekutor has started.");

	PeisTuple paramTuple = getParamTuple();

	char params_str[256];
	strcpy(params_str, paramTuple.data);

	// Cartesian space or Joint space.
	std::string space_str = strtok (params_str," ,");

	for(int i = 0; i<space_str.size(); i++)
		space_str[i] = tolower(space_str[i]);

	// Relative or Absolute command.
	std::string type_str = strtok(NULL, " ,");

	for(int i = 0; i<type_str.size(); i++)
		type_str[i] = tolower(type_str[i]);

	// Get whatever remains.
	char* params_ptr = strtok(NULL, "");

	std::vector<double> the_values = extractParams(params_ptr);

	ROS_INFO("Parameters obtained...");

	int num_arg = the_values.size();

	if(space_str.compare("joints") != 0 && space_str.compare("cartesian") != 0)
	{
		ROS_INFO("The first parameter ('%s') should be either 'cartesian' or 'joints'. Aborting action.", space_str.c_str());
		setState(FAILED);
		return;
	}

	if(type_str.compare("absolute") != 0 && type_str.compare("relative") != 0)
	{
		ROS_INFO("The second parameter ('%s') should be either 'absolute' or 'relative'. Aborting action.", type_str.c_str());
		setState(FAILED);
		return;
	}

	if(num_arg != 6)
	{
		ROS_INFO("The number of arguments should be exactly six - six joint angles or x, y, z, r, p, y. Aborting action.");
		setState(FAILED);
		return;
	}

	// All parameters are valid. Now we can send goals.

	if(space_str.compare("cartesian") == 0)
	{
		ROS_INFO("Given goal is a cartesian-space goal.");
		jaco::CartesianMovementGoal test_goal;

		// If the type parameter is relative, set frame to hand_jaco, i.e., the current pose of the arm.
		// Else, it is set to base_jaco, which signifies that the pose is w.r.t. the base of jaco.
		test_goal.poseGoal.header.frame_id = (type_str.compare("relative") == 0) ? "hand_jaco" : "base_jaco";

		test_goal.poseGoal.position.x = the_values[0];
		test_goal.poseGoal.position.y = the_values[1];
		test_goal.poseGoal.position.z = the_values[2];

		test_goal.poseGoal.orientation.x = the_values[3];
		test_goal.poseGoal.orientation.y = the_values[4];
		test_goal.poseGoal.orientation.z = the_values[5];

		cartesian_client_.waitForServer();
		ROS_INFO("Sending goal...");
		cartesian_client_.sendGoal(test_goal);

		cartesian_client_.waitForResult();

		if(cartesian_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Successfully executed the action on the robot.");
			setState(COMPLETED);
			sleep(1);
		}
		else
		{
			ROS_INFO("\n\nAttempted! But, something cocked-up and we failed!");
			setState(FAILED);
			sleep(1);
		}
	}

	// else if space_str is 'joints'.
	else
	{
		ROS_INFO("Given goal is a joint-space goal.");
		control_msgs::FollowJointTrajectoryGoal joint_goal;

		joint_goal.trajectory.header.stamp = ros::Time::now();

		// 'jaco_joint_1', 'jaco_joint_2', 'jaco_joint_3', 'jaco_joint_4', 'jaco_joint_5', 'jaco_joint_6'
		joint_goal.trajectory.joint_names.push_back("jaco_joint_1");
		joint_goal.trajectory.joint_names.push_back("jaco_joint_2");
		joint_goal.trajectory.joint_names.push_back("jaco_joint_3");

		joint_goal.trajectory.joint_names.push_back("jaco_joint_4");
		joint_goal.trajectory.joint_names.push_back("jaco_joint_5");
		joint_goal.trajectory.joint_names.push_back("jaco_joint_6");

		std::vector<double> _points_;
		_points_.push_back(the_values[0]);
		_points_.push_back(the_values[1]);
		_points_.push_back(the_values[2]);
		_points_.push_back(the_values[3]);
		_points_.push_back(the_values[4]);
		_points_.push_back(the_values[5]);

		joint_goal.trajectory.points = std::vector<trajectory_msgs::JointTrajectoryPoint>(1);
		joint_goal.trajectory.points[0].positions = _points_;

		if(type_str.compare("relative") == 0)
		{
			ROS_INFO("Updating message to include current joint-pose...");
			joint_states_sub_ = nh_.subscribe("jaco/joint_states", 1, &MoveHandExekutor::jointStatesCallback, this);

			pthread_t spin_one_sec_thread;
			pthread_create(&spin_one_sec_thread, NULL, spin_once, NULL);

			while(current_joint_state_.position.size() == 0)
				usleep(100000);

			pthread_cancel(spin_one_sec_thread);

			joint_states_sub_.shutdown();



			joint_goal.trajectory.points[0].positions[0] += current_joint_state_.position[0];
			joint_goal.trajectory.points[0].positions[1] += current_joint_state_.position[1];
			joint_goal.trajectory.points[0].positions[2] += current_joint_state_.position[2];
			joint_goal.trajectory.points[0].positions[3] += current_joint_state_.position[3];
			joint_goal.trajectory.points[0].positions[4] += current_joint_state_.position[4];
			joint_goal.trajectory.points[0].positions[5] += current_joint_state_.position[5];
		}

		ROS_INFO("Final joint values are: [%lf, %lf, %lf, %lf, %lf, %lf]",
				joint_goal.trajectory.points[0].positions[0],
				joint_goal.trajectory.points[0].positions[1],
				joint_goal.trajectory.points[0].positions[2],
				joint_goal.trajectory.points[0].positions[3],
				joint_goal.trajectory.points[0].positions[4],
				joint_goal.trajectory.points[0].positions[5]);

		joints_client_.waitForServer();
		ROS_INFO("Sending goal...");
		joints_client_.sendGoal(joint_goal);

		joints_client_.waitForResult();

		if(joints_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Successfully executed the action on the robot");
			setState(COMPLETED);
			sleep(1);
		}
		else
		{
			ROS_INFO("\n\nAttempted! But, something cocked-up and we failed!");
			setState(FAILED);
			sleep(1);
		}

	}

}

} /* namespace exekutor */
