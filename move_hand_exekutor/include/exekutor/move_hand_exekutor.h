/*
 * move_hand_exekutor.h
 *
 *  Created on: Jul 2, 2014
 *      Author: ace
 */

#ifndef MOVE_HAND_EXEKUTOR_H_
#define MOVE_HAND_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <jaco/CartesianMovementAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

namespace exekutor {

/**
 * Exekutor that moves the arm either in the joint-space or in the cartesian space.
 * Co-ordinates can be incremental or absolute.
 */
class MoveHandExekutor: public ActionExekutor
{
protected:

	/**
	 * A client to the 'jaco/jaco_arm_controller/joint_trajectory_action' action server.
	 */
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joints_client_;

	/**
	 * A client to the 'jaco/cartesian_action' action server.
	 */
	actionlib::SimpleActionClient<jaco::CartesianMovementAction> cartesian_client_;

	/**
	 * A subscriber to listen to the 'jaco/joint_states' topic.
	 */
	ros::Subscriber joint_states_sub_;

	/**
	 * A callback for the joint_states subscriber.
	 */
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr& _msg);

	/**
	 * A variable to hold the joint_states value.
	 */
	sensor_msgs::JointState current_joint_state_;

	/**
	 * The actual function that is called when tuple goes ON.
	 * Here we create clients to 'jaco/cartesian_action' or to
	 * 'jaco/jaco_arm_controller/joint_trajectory_action'
	 *
	 * The implementation of actionThread() function.
	 * This is called from ActionExekutor's startActionThread().
	 */
	virtual void actionThread();

public:
	/**
	 * A constructor that takes in the name of the robot and the name of the action.
	 */
	MoveHandExekutor(std::string robot_name, std::string action_name);

	/**
	 * Destructor.
	 */
	virtual ~MoveHandExekutor();
};

} /* namespace exekutor */

#endif /* MOVE_HAND_EXEKUTOR_H_ */
