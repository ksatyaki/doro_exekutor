/*
 * moveit_hand_exekutor_test.cc
 *
 *  Created on: Sep 5, 2014
 *      Author: ace
 */

extern "C"
{
#include <peiskernel/peiskernel_mt.h>
}

#include <exekutor/moveit_hand_exekutor.h>
#include <signal.h>


void handler(int signal_number)
{
	if(signal_number == SIGINT)
		printf("Ctrl-C caught...\n");
	else
		printf("Ctrl-Z caught...\n");
	ros::shutdown();

	abort();
}

int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "moveit_hand_exekutor_test");

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);

	exekutor::MoveitHandExekutor moveit_hand_exek ("Doro", "MoveitHand");
	exekutor::ActionExekutor::waitForLink();

	return 0;
}



