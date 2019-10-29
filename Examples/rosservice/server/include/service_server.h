

#ifndef SERVICE_SERVER_H
#define SERVICE_SERVER_H

#include <ros/ros.h>
#include "tutorial_msgs/ControlMode.h"
#include <typeinfo>   // operator typeid
#include <vector>
using namespace std;


class Mode
{

    private:

        bool changeControlMode(tutorial_msgs::ControlMode::Request  &req,
                                 tutorial_msgs::ControlMode::Response &res);        

    public:
 		Mode(ros::NodeHandle nh);        
        ros::ServiceServer control_mode_service_;
};


void getControlMode(std::vector<int> const &input);  

#endif

