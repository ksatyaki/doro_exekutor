/*
 * moveit_hand_exekutor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: ace
 */

#ifndef MOVEIT_HAND_EXEKUTOR_H_
#define MOVEIT_HAND_EXEKUTOR_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <exekutor/action_exekutor.h>
#include <actionlib/client/simple_action_client.h>

#include <doro_manipulation/PlanAndMoveArmAction.h>

namespace exekutor
{

/**
 * PEIS-interface to plan and move to a pose in 3D-space using Moveit.
 */
class MoveitHandExekutor : public ActionExekutor
{
protected:

	/**
	 * Action client to call the plan_and_move server.
	 */
	actionlib::SimpleActionClient <doro_manipulation::PlanAndMoveArmAction> pam_client_;

	/**
	 * The implementation of actionThread().
	 */
	virtual void actionThread();

public:
	/**
	 * Constructor.
	 */
	MoveitHandExekutor(std::string robot_name, std::string action_name);

	/**
	 * Destructor.
	 */
	virtual ~MoveitHandExekutor();
};

} /* namespace exekutor */

#endif /* MOVEIT_HAND_EXEKUTOR_H_ */
