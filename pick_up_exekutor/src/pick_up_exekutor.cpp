/*
 * pick_up_exekutor.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: ace
 */

#include "exekutor/pick_up_exekutor.h"

namespace exekutor
{

PeisTuple getStateTuple(std::string partial_key)
{
	std::string state_key (partial_key + ".state");
	PeisTuple* state_tuple = peiskmt_getTuple(peiskmt_peisid(), state_key.c_str(), PEISK_KEEP_OLD);
	while(!state_tuple)
	{
		state_tuple = peiskmt_getTuple(peiskmt_peisid(), state_key.c_str(), PEISK_KEEP_OLD);
		sleep(1);
	}

	return *state_tuple;
}

void PickUpExekutor::stateCallback(PeisTuple* recd_tuple, void* arg)
{
	ROS_INFO("We saw the state change. Current State: %s", recd_tuple->data);

	if(strcmp(recd_tuple->data, "COMPLETED") == 0)
		((PickUpExekutor *)arg)->stateValue = COMPLETED;
	else if(strcmp(recd_tuple->data, "FAILED") == 0)
		((PickUpExekutor *)arg)->stateValue = FAILED;
	else if(strcmp(recd_tuple->data, "RUNNING") == 0)
		((PickUpExekutor *)arg)->stateValue = RUNNING;
}

PickUpExekutor::PickUpExekutor(std::string robot_name, std::string action_name):
		ActionExekutor(robot_name, action_name),
		pam_client_("plan_and_move_arm", true)
{
	gpg_client_ = nh_.serviceClient <doro_manipulation::GenerateGraspPoses> ("generate_grasp_poses", false);
}

PickUpExekutor::~PickUpExekutor()
{

}

void PickUpExekutor::actionThread()
{
	ROS_INFO("PickUpExekutor has started!");

	std::string parameter = getParamTuple().data;

	// Start by calling Look.

	meta_tuples_linker::MetaTuplesLinker our_linker (my_peis_id);

	our_linker.setTuples("pick_up.look.1", "ON", parameter, "IDLE");
	our_linker.setTuples("pick_up.acquire.2", "ON", parameter, "IDLE");
	our_linker.setTuples("pick_up.fiddle.4", "ON", "close", "IDLE");
	our_linker.setTuples("pick_up.moveithand.5", "ON", "-0.143 1.064 -0.136 0.656 -0.227 0.244 0.678", "IDLE");

	/**
	Translation: [-0.143, 1.064, -0.136]
	- Rotation: in Quaternion [0.656, -0.227, 0.244, 0.678]
	**/
	stateValue = RUNNING;

	// LINK 1
	ROS_INFO("Doro-Look");
	our_linker.link("pick_up.look.1", "doro.look");
	peiskmt_setStringTuple("out.doro.look.state", "IDLE");
	PeisCallbackHandle cHandle = peiskmt_registerTupleCallback(my_peis_id, "out.doro.look.state", (void *)this, &PickUpExekutor::stateCallback);
	usleep(100000);
	while(stateValue != COMPLETED && stateValue != FAILED)
	{
		usleep(500000);
	}
	peiskmt_unregisterTupleCallback(cHandle);

	// LINK 2
	ROS_INFO("Doro-Acquire");
	our_linker.link("pick_up.acquire.2", "doro.acquire");
	peiskmt_setStringTuple("out.doro.acquire.state", "IDLE");
	stateValue = RUNNING;
	cHandle = peiskmt_registerTupleCallback(my_peis_id, "out.doro.acquire.state", (void *)this, &PickUpExekutor::stateCallback);
	usleep(100000);
	while(stateValue != COMPLETED && stateValue != FAILED)
	{
		usleep(500000);
	}
	peiskmt_unregisterTupleCallback(cHandle);

	// LINK 3
	ROS_INFO("Calling Grasp Pose Generator.");
	doro_manipulation::GenerateGraspPoses message;
	message.request.object_location = cam_interface::getObjectPositionFromCAM(parameter, this->tf_listener_);

	if(gpg_client_.call(message))
	{
		ROS_INFO("Got the grasp poses.");
	}

	else
	{
		setState(FAILED);
		ROS_INFO("Grasp Pose Generation has failed somehow. Action Failed!");
		return;
	}

	// LINKS 3b Open hand
	ROS_INFO("Doro-Fiddle");
	our_linker.setTuples("pick_up.fiddle.3b", "ON", "open", "IDLE");
	our_linker.link("pick_up.fiddle.3b", "doro.fiddle");
	peiskmt_setStringTuple("out.doro.fiddle.state", "IDLE");
	stateValue = RUNNING;
	cHandle = peiskmt_registerTupleCallback(my_peis_id, "out.doro.fiddle.state", (void *)this, &PickUpExekutor::stateCallback);
	usleep(100000);
	while(stateValue != COMPLETED && stateValue != FAILED)
	{
		usleep(500000);
	}
	if(stateValue == FAILED)
	{
		setState(FAILED);
		ROS_INFO("Gripper open has failed somehow. Action Failed!");
		return;
	}
	peiskmt_unregisterTupleCallback(cHandle);


	// LINKS 4
	ROS_INFO("Moving arm to grasp.");

	bool move_hand_success = false;
	for(int i = 0; i < message.response.grasp_poses.size(); i++)
	{
		geometry_msgs::PoseStamped& pregrasp_pose = message.response.pregrasp_poses[i];
		char pregrasp_pose_param[80];
		sprintf(pregrasp_pose_param, "%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f",
				pregrasp_pose.pose.position.x,
				pregrasp_pose.pose.position.y,
				pregrasp_pose.pose.position.z,
				pregrasp_pose.pose.orientation.x,
				pregrasp_pose.pose.orientation.y,
				pregrasp_pose.pose.orientation.z,
				pregrasp_pose.pose.orientation.w);

		ROS_INFO("Doro-MoveitHand");
		our_linker.setTuples("pick_up.moveit.4", "ON", pregrasp_pose_param, "IDLE");
		our_linker.link("pick_up.moveit.4", "doro.moveithand");
		peiskmt_setStringTuple("out.doro.moveithand.state", "IDLE");
		stateValue = RUNNING;
		cHandle = peiskmt_registerTupleCallback(my_peis_id, "out.doro.moveithand.state", (void *)this, &PickUpExekutor::stateCallback);
		usleep(100000);
		while(stateValue != COMPLETED && stateValue != FAILED)
		{
			usleep(500000);
		}
		peiskmt_unregisterTupleCallback(cHandle);
		if(stateValue == FAILED)
		{
			if(i == message.response.grasp_poses.size() - 1)
			{
				setState(FAILED);
				ROS_INFO("Somehow pick up failed because of the actual picking up. See moveit. Action Failed!");
				return;
			}
			else
				continue;
		}

		geometry_msgs::PoseStamped& grasp_pose = message.response.grasp_poses[i];
		char pose_param[80];
		sprintf(pose_param, "%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f",
				grasp_pose.pose.position.x,
				grasp_pose.pose.position.y,
				grasp_pose.pose.position.z,
				grasp_pose.pose.orientation.x,
				grasp_pose.pose.orientation.y,
				grasp_pose.pose.orientation.z,
				grasp_pose.pose.orientation.w);

		ROS_INFO("Doro-MoveitHand");
		our_linker.setTuples("pick_up.moveit.5", "ON", pose_param, "IDLE");
		our_linker.link("pick_up.moveit.5", "doro.moveithand");
		peiskmt_setStringTuple("out.doro.moveithand.state", "IDLE");
		stateValue = RUNNING;
		cHandle = peiskmt_registerTupleCallback(my_peis_id, "out.doro.moveithand.state", (void *)this, &PickUpExekutor::stateCallback);
		usleep(100000);
		while(stateValue != COMPLETED && stateValue != FAILED)
		{
			usleep(500000);
		}
		peiskmt_unregisterTupleCallback(cHandle);
		if(stateValue == FAILED)
		{
			if(i == message.response.grasp_poses.size() - 1)
			{
				setState(FAILED);
				ROS_INFO("Somehow pick up failed because of the actual picking up. See moveit. Action Failed!");
				return;
			}
			else
				continue;
		}
		else
			break;
	}

	// LINK 5
	ROS_INFO("Doro-Fiddle");
	our_linker.link("pick_up.fiddle.4", "doro.fiddle");
	peiskmt_setStringTuple("out.doro.fiddle.state", "IDLE");
	stateValue = RUNNING;
	cHandle = peiskmt_registerTupleCallback(my_peis_id, "out.doro.fiddle.state", (void *)this, &PickUpExekutor::stateCallback);
	usleep(100000);
	while(stateValue != COMPLETED && stateValue != FAILED)
	{
		usleep(500000);
	}
	peiskmt_unregisterTupleCallback(cHandle);
	if(stateValue == FAILED)
	{
		setState(FAILED);
		ROS_INFO("Gripper close has failed somehow. Action Failed!");
		return;

	}

	// LINK 6

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

	setState(COMPLETED);
	ROS_INFO("Action succeeded.");
}

} /* namespace exekutor */
