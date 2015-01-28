/*
 * dock_exekutor.h
 *
 *  Created on: Sep 3, 2014
 *      Author: ace
 */

#ifndef DOCK_EXEKUTOR_H_
#define DOCK_EXEKUTOR_H_

#include <exekutor/action_exekutor.h>
#include <cam_interface/cam_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <simple_service/MoveToSimpleAction.h>

namespace exekutor {

enum operation
{
	UNDOCK,
	DOCK
};

/**
 * Dock to an object to be able to manipulate it.
 * The position of the object is assumed to be stored in the tuple: CAM.objects.object_name.position
 */

class DockExekutor: public ActionExekutor
{

	/**
	 * A variable to hold the last action performed.
	 */
	operation last_operation;

	/**
	 * Store the previous dock_goal after the pose is undone.
	 */
	geometry_msgs::PoseStamped undock_goal;

	/**
	 * A simple_action_client to the 'move_to_simple' action server.
	 */
	actionlib::SimpleActionClient <simple_service::MoveToSimpleAction> move_to_simple_client_;

	/**
	 * This function creates a client to the move_to_simple action and uses it to dock the robot.
	 *
	 * The implementation of actionThread() function.
	 * This is called from ActionExekutor's startActionThread().
	 * This is where we create the client to the move_to_simple action.
	 */
	void actionThread();

	/**
	 * Sequence of things to do when we are asked to dock.
	 */
	void dockAction(const std::string& object_name);

	/**
	 * Sequence of things to do when we are asked to undock.
	 */
	void undockAction();

public:
	DockExekutor(std::string robot_name, std::string action_name);
	virtual ~DockExekutor();
};

} /* namespace exekutor */

#endif /* DOCK_EXEKUTOR_H_ */
