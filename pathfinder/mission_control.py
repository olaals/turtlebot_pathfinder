#! /usr/bin/env python

import rospy
import actionlib

from pathfinder.msg import calculate_pointAction, calculate_pointGoal


class MissionControl:

    def __init__(self):
        
        self.nav_client = actionlib.SimpleActionClient("nav_server", calculate_pointAction)
        self.nav_client.wait_for_server()

    def calc_point(self):
        
        calc_point_goal = calculate_pointGoal()
        self.nav_client.send_goal(calc_point_goal)
        self.nav_client.wait_for_result()

        return self.nav_client.get_result()
    
    def main(self):
        point = self.calc_point()
        rospy.loginfo(point)


if __name__ == "__main__":
    rospy.init_node("mission_control")
    mission_control = MissionControl()
    mission_control.main()