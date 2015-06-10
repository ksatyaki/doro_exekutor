#include <exekutor/look_exekutor.h>
#include <exekutor/acquire_exekutor.h>
#include <exekutor/dock_exekutor.h>
#include <exekutor/moveit_hand_exekutor.h>
#include <exekutor/fiddle_exekutor.h>
#include <exekutor/move_hand_exekutor.h>
#include <exekutor/move_to_exekutor.h>
#include <exekutor/move_to_simple_exekutor.h>
#include <exekutor/pick_up_exekutor.h>
#include <exekutor/handover_exekutor.h>
#include <fake_exekutor.h>
#include <exekutor/handle_exekutor.h>
#include <cam_interface/cam_interface.h>

#include<signal.h>

void handler(int signal_number)
{
	if(signal_number == SIGINT)
	{
		printf("Ctrl-C caught...\n");
		ros::shutdown();
		peiskmt_shutdown();
		abort();

	}
	else
	{
		printf("Ctrl-Z caught...\n");
		ros::shutdown();
		peiskmt_shutdown();
		abort();
	}
}


int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "exekutor");
	
	cam_interface::subscribeToCAM();

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);
	
	exekutor::LookExekutor LX("doro", "look");
	exekutor::DockExekutor DX("doro", "dock");
	exekutor::AcquireExekutor AX("doro", "acquire");
	exekutor::MoveitHandExekutor MHX("doro", "moveithand");
	exekutor::FiddleExekutor FX("doro", "fiddle");
	exekutor::MoveHandExekutor MIHX("doro", "movehand");
	exekutor::MoveToExekutor MTX("doro", "moveto");
	exekutor::MoveToSimpleExekutor MTSX("doro", "movetosimple");
	exekutor::PickUpExekutor PU("doro", "pickup");
	exekutor::HandoverExekutor HO("doro", "handover");
	exekutor::FakeExekutor FE("doro", "wait");
	exekutor::HandleExekutor HX("doro", "handle");
	
	//ROS_INFO("Printing all.");
	//exekutor::ActionExekutor::printAll();
	//ROS_INFO("Printing all done.");
	
	exekutor::ActionExekutor::waitForLink();
	
	return 0;
}

