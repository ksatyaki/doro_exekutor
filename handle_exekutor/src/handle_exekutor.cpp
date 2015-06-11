#include <exekutor/handle_exekutor.h>

namespace exekutor {
	HandleExekutor::HandleExekutor (std::string robot_name, std::string action_name) :
		ActionExekutor(robot_name, action_name) {
		last_status_ = false;
		client_ = nh_.serviceClient <led_handle_board::GrabControl> ("GrabControl");
	}

	void HandleExekutor::handleStatusCallback(const std_msgs::BoolConstPtr& in_status) {
	  
	  last_status_ = current_status_;
	  current_status_ = in_status->data;

	}

	void HandleExekutor::actionThread() {

		led_handle_board::GrabControl msg;
		msg.request.on = true;
		if(client_.call(msg)) {
			ROS_INFO("Handle Control has started!");
		}
		else {
			ROS_INFO("Can't call the handle program. Contact Manzi.");
			setState (FAILED);
			return;
		}

		this->handle_status_sub_ = nh_.subscribe("/grab_is_active", 1, &HandleExekutor::handleStatusCallback, this);

		ROS_INFO("Waiting for a timeout!");
		while(current_status_ || !last_status_) {
		  ros::spinOnce();
		}
		this->handle_status_sub_.shutdown();
		ROS_INFO("Sensed timeout!");
		ROS_INFO("Success!");
		setState(COMPLETED);
	}
}
