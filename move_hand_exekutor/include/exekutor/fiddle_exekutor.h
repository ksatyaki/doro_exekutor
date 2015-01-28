/*
 * fiddle_exekutor.h
 *
 *  Created on: Jul 10, 2014
 *      Author: ace
 */

#ifndef FIDDLE_EXEKUTOR_H_
#define FIDDLE_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>

#include <actionlib/client/simple_action_client.h>
#include <jaco/FingerMovementAction.h>

namespace exekutor {

/**
 * The exekutor that controls the fingers of the jaco-arm.
 */
class FiddleExekutor: public ActionExekutor
{
	/**
	 * Simple client to the finger movement action server.
	 */
	actionlib::SimpleActionClient <jaco::FingerMovementAction> finger_action_client_;

	/**
	 * The actual function that is called when tuple goes ON.
	 * Here we create a client to 'jaco/finger_action'.
	 * This is called from ActionExekutor's startActionThread().
	 */
	virtual void actionThread();

public:
	/**
	 * A constructor that takes in the name of the robot and the name of the action.
	 */
	FiddleExekutor(std::string robot_name, std::string action_name);

	/**
	 * Destructor.
	 */
	virtual ~FiddleExekutor();
};

} /* namespace exekutor */

#endif /* FIDDLE_EXEKUTOR_H_ */
