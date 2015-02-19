/*
 * pick_up_exekutor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: ace
 */

#ifndef PICK_UP_EXEKUTOR_H_
#define PICK_UP_EXEKUTOR_H_

#include <meta_tuples_linker.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <exekutor/action_exekutor.h>
#include <cam_interface/cam_interface.h>

#include <doro_manipulation/PlanAndMoveArmAction.h>
#include <doro_manipulation/GenerateGraspPoses.h>

namespace exekutor
{

class PickUpExekutor: public ActionExekutor
{
	/**
	 * A client to call the generate_grasp_poses server.
	 */
	ros::ServiceClient gpg_client_;

	/**
	 * A plan and move arm client.
	 */
	actionlib::SimpleActionClient<doro_manipulation::PlanAndMoveArmAction> pam_client_;

	/**
	 * The implementation of actionThread().
	 */
	virtual void actionThread();

	/**
	 * A PeisCallback function for status callback.
	 */
	static void stateCallback(PeisTuple* recd_tuple, void* arg);

	/**
	 * Keep track of dispatched activities.
	 */
	StateValue stateValue;

public:
	/**
	 * Constructor.
	 */
	PickUpExekutor(std::string robot_name, std::string action_name);

	/**
	 * Destructor.
	 */
	virtual ~PickUpExekutor();
};

} /* namespace exekutor */

#endif /* PICK_UP_EXEKUTOR_H_ */
