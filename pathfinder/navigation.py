#! /usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import Point

# import needed messages defined in actions
from pathfinder.msg import get_envAction, get_envGoal, calculate_pointAction, calculate_pointResult, calculate_pointFeedback

class Navigator:

    # define messages for action server
    _feedback = calculate_pointFeedback()
    _result = calculate_pointResult()

    def __init__(self):

        # Set up client to situational awareness
        self.client = actionlib.SimpleActionClient('env_server', get_envAction)
        self.client.wait_for_server()

        # Set up server for mission control 
        self.calc_server = actionlib.SimpleActionServer('nav_server', calculate_pointAction, self.calc_point_cb, False)
        self.calc_server.start()


    def calc_point_cb(self, goal):
        env = self.get_env()

        # tmp show of usage
        self._result.get_point.x = env.decision_space[0]
        self._result.get_point.y = env.decision_space[1]

        # "send" result 
        self.calc_server.set_succeeded(self._result)



    def get_env(self):
        
        # define goal/action that action server should execute
        goal = get_envGoal()

        # send goal to action server
        self.client.send_goal(goal)

        # wait for server to finish action
        self.client.wait_for_result()

        # get and use result
        return self.client.get_result()
    
    def main(self):
        env = self.get_env()
        rospy.loginfo(env)


if __name__ == "__main__":
    rospy.init_node('navigator')
    navigator = Navigator()
    navigator.main()