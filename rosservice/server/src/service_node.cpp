

#include "service_server.h"

Mode::Mode(ros::NodeHandle nh_){

    /** Services **/
    control_mode_service_ = nh_.advertiseService("controlmode_service",&Mode::changeControlMode, this);
	ROS_INFO("Ready to change mode");

}

void getControlMode(std::vector<bool> const &input){

	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ';
	}
}


bool Mode::changeControlMode(tutorial_msgs::ControlMode::Request  	&req,
                               tutorial_msgs::ControlMode::Response &res)
{
	ROS_INFO("entered callback");

	cout << "mode: " << req.ControlMode << endl;

	try {
		//getControlMode(mode);
		res.mode = "success";
		ROS_INFO("successfull callback");
	} catch (const std::exception& e){
		res.mode = "failed";
		ROS_INFO("failed callback");
	}
	return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ControlMode");
  ros::NodeHandle nh;
  Mode Mode(nh);

	while(ros::ok()){
		ros::spinOnce();
	}

	return 0;
}
