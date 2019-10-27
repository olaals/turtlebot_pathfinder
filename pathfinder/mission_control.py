#! /usr/bin/env python

import rospy
import actionlib

from pathfinder.msg import calculate_pointAction, calculate_pointGoal, go_to_pointAction, go_to_pointGoal


class MissionControl:

    def __init__(self):
        
        rospy.loginfo("Setting up client for nav_server...")
        self.nav_client = actionlib.SimpleActionClient("nav_server", calculate_pointAction)
        self.nav_client.wait_for_server()
        rospy.loginfo("Client for nav_server up and running")
        
        rospy.loginfo("Setting up client for vehicle_server...")
        self.vehicle_client = actionlib.SimpleActionClient("vehicle_server", go_to_pointAction)
        self.vehicle_client.wait_for_server()
        rospy.loginfo("Client for vehicle_server up and running")


    def calc_point(self):
        
        calc_point_goal = calculate_pointGoal()
        self.nav_client.send_goal(calc_point_goal)
        self.nav_client.wait_for_result()

        return self.nav_client.get_result()
    
    def main(self):
        point = self.calc_point()
        rospy.loginfo(point)

        goal_twist = go_to_pointGoal()
        goal_twist.goal_twist.linear.x = 1

        self.vehicle_client.send_goal(goal_twist)
        rospy.loginfo("Waiting for vehicle controller...")

        self.vehicle_client.wait_for_result()
        res = self.vehicle_client.get_result()

        rospy.loginfo(res)


if __name__ == "__main__":
    rospy.init_node("mission_control")
    mission_control = MissionControl()
    mission_control.main()