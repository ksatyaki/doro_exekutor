#include <exekutor/action_exekutor.h>
#include <led_handle_board/GrabControl.h>
#include <std_msgs/Bool.h>

namespace exekutor {
	class HandleExekutor : public ActionExekutor
	{	
	protected:
		ros::Subscriber handle_status_sub_;

		bool last_status_;

		bool current_status_;

		ros::ServiceClient client_;

		void handleStatusCallback(const std_msgs::BoolConstPtr& in_status);
		
		/** 
		 * The overloaded actionThread function.
		 */
		void actionThread();
		
	public:
		HandleExekutor(std::string robot_name = "doro", std::string action_name = "handle");
	};	
}
