#include <exekutor/pick_up_exekutor.h>
#include <ros/ros.h>

extern "C"
{
#include <peiskernel/peiskernel_mt.h>
}

int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "pick_up_exekutor_node");

	exekutor::PickUpExekutor cute_tor ("Doro", "PickUp");
	exekutor::ActionExekutor::waitForLink();

	return 0;
}
