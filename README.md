======
README	   
======

This document describes how to install/use the exekutor interface.

		Author: 		Chittaranjan Srinivas Swaminathan
		Instituition: 	Orebro University


###BRIEF DESCRIPTION

Exekutor currently has four capabilities.

1. MoveToExekutor - Uses the ros move_base action server to send robot to a goal on the map.

	Parameter format: 
		
		x y [theta [xy_tolerance [yaw_tolerance]]]
		
		Theta is the heading in radian.
		xy_tolerance, x and y are in metres.
		yaw_tolerance is in radian.
	
Depends:

ROS PACKAGES:
	action_exekutor
	actionlib
	move_base_msgs
	 
2. MoveToSimpleExekutor - Uses only the transform information between odom and base_link frames to move the base. That is, it is a simple odometry based move action. CAUTION: It does not use obstacle avoidance.

	Parameter format:
		
		x y [theta [xy_tolerance [yaw_tolerance]]]
		
		Theta is the heading in radian.
		xy_tolerance, x and y are in metres.
		yaw_tolerance is in radian.

Depends:

ROS PACKAGES:
	action_exekutor
	simple_service

3. AcquireExekutor - Uses euclidean cluster extraction along with sift descriptor matching to 'acquire' the scene in front of the robot.
This serves the purpose of Anchoring.

	Parameter format:
		
		<name_of_object> [tolerance_percent]
		
		or
		
		meta_object [tolerance_percent]
		(in this case the an object should be created in CAM with name exactly "meta_object" and with signature values filled in).
		
		or 
		
		known [tolerance_percent]
		
		or 
		
		all [tolerance_percent]
		
	Typically all the objects in scene are anchored no matter which version of acquire is called. The tolerance_percent is a measure of how strict or lenient the anchoring process should be (1 - strict, ~0 - lenient).
		
Build depends:

ROS PACKAGES:
	action_exekutor
	doro_msgs
	acquire_tabletop
	
Run depends:

ROS_PACKAGES:
	cluster_extraction
	acquire_tabletop

PEIS COMPONENTS:
	CAM

4. LookExekutor

	Parameter format:
	
		<name_of_object>

5. DockExekutor

	Parameter format:
		
		dock <name_of_object>
		
		or
		
		undock

6. MoveHandExekutor

	Parameter format:
	
		cartesian/joints absolute/relative _1 _2 _3 _4 _5 _6

7. FiddleExekutor

	Parameter format:
		
		open
		
		or 
		
		close
		
		or
		
		absolute finger_1 finger_2 finger_3

8. MoveitHandExekutor

	Paramter format:
	
		p.x p.y p.z o.x o.y o.z o.w
		
		where p is the position and o is the orientation. All 7 values are required.
		Assumes the point is within the workspace. Would fail otherwise.

9. PickUpExekutor

	Parameter format:
		
		<name_of_object>
	
###INSTALLATION

####STEP 1:
ENSURE the latest version of PEIS is installed.

####STEP 2:
ROS up-to-date?

Do,

sudo apt-get update
sudo apt-get dist-upgrade


p.s. Don't start swearing already. You don't really have to do a dist-upgrade :P .... Just ensure ros-hydro-desktop-full is up-to-date.

Also install the python tool wstool.

	sudo pip install wstool
	
	or
	
	sudo apt-get install python-wstool

See http://wiki.ros.org/wstool for details.

####STEP 3:
Getting dependencies:

Assuming you are now in the catkin_workspace/src folder,

	wstool init .
	wstool add simple_service --git git://github.com/ksatyaki/simple_service.git
	wstool add simple_service --git git://github.com/ksatyaki/acquire_tabletop.git
	wstool add simple_service --git git://github.com/ksatyaki/cam_interface.git
	wstool add simple_service --git git://github.com/ksatyaki/doro_manipulation.git
	wstool add doro_msgs --git git://github.com/ksatyaki/doro_msgs.git
	wstool add exekutor --git git://github.com/ksatyaki/exekutor.git
	wstool update

####STEP 5:
Now we 'make' the catkin_workspace

	cd ..
	
	catkin_make
	

If this fails because a dependent package can't be found, then ensure that the following packages are installed:

1. ros-hydro-move-base
2. ros-hydro-move-base-msgs
3. ros-hydro-actionlib
4. ros-hydro-actionlib-msgs

####STEP 6:
INSTALL test_exekutor

Unzip test_exekutor to another directory and type:

	cmake .

	make
