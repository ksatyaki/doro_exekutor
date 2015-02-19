#include <exekutor/handover_exekutor.h>
#include <ros/ros.h>

extern "C"
{
#include <peiskernel/peiskernel_mt.h>
}

int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "handover_exekutor_test_node");

	exekutor::HandoverExekutor cute_tor ("doro", "handover");
	exekutor::ActionExekutor::waitForLink();

	return 0;
}
