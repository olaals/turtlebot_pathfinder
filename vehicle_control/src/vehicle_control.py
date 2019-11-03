#! /usr/bin/env python 

import rospy
import actionlib
from geometry_msgs.msg import Pose, Twist

from tutorial_msgs.msg import go_to_pointAction, go_to_pointResult


class VehicleControl:
    """
    Controller that moves vehicle to desired pose. 
    Communicates with turtlebot through Twist messages. 
    """

    _result = go_to_pointResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer("vehicle_server", go_to_pointAction, self.go_to_point_cb, False)
        self.server.start()

    def go_to_point_cb(self, goal):
        # TODO: implement controller that moves vehicle to goal pose
        
        self._result.success = 1
        self.server.set_succeeded(self._result)
        # TODO: change success variable to succeeded/aborted


if __name__ == "__main__":
    rospy.init_node("vehicle_control")
    vehicle_control = VehicleControl()
    rospy.spin()
