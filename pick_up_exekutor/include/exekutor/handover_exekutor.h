/*
 * handover_exekutor.h
 *
 *  Created on: Feb 19, 2015
 *      Author: ace
 */

#ifndef DORO_EXEKUTOR_PICK_UP_EXEKUTOR_INCLUDE_HANDOVER_EXEKUTOR_H_
#define DORO_EXEKUTOR_PICK_UP_EXEKUTOR_INCLUDE_HANDOVER_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <doro_manipulation/PlanAndMoveArmAction.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

namespace exekutor {

class HandoverExekutor: public ActionExekutor
{
	/**
	 * A plan and move arm client.
	 */
	actionlib::SimpleActionClient<doro_manipulation::PlanAndMoveArmAction> pam_client_;

	/**
	 * The implementation of actionThread().
	 */
	virtual void actionThread();

	/**
	 * A callback for the jaco/joint_states topic.
	 */
	void jacoJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

	/**
	 * Keeping track.
	 */
	boost::shared_ptr<double> prev_joint_3_value, current_joint_3_value;

	/**
	 * A publisher to publish the finger command.
	 */
	ros::Publisher fingers_pub_;

	/**
	 * A message to open gripper. Set in constructor.
	 */
	std_msgs::Float64MultiArray release_msg;

	// Just publish the release msg to the proper topic.
	inline void releaseHold()
	{
		fingers_pub_.publish(release_msg);
	}

public:
	HandoverExekutor(std::string robot_name, std::string action_name);

	virtual ~HandoverExekutor();
};

} /* namespace exekutor */

#endif /* DORO_EXEKUTOR_PICK_UP_EXEKUTOR_INCLUDE_HANDOVER_EXEKUTOR_H_ */
