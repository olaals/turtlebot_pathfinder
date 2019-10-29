#! /usr/bin/env python

import rospy
import actionlib

from tutorial_msgs.msg import get_envFeedback, get_envResult, get_envAction


class SituationalAwareness:

    # define messages for action server
    _feedback = get_envFeedback()
    _result = get_envResult()

    def __init__(self):

        # Create server for nicer data extraction    
        self.env_server = actionlib.SimpleActionServer("env_server", get_envAction, self.get_env_cb, False)
        self.env_server.start()

        # Create client for collision avoidance
        # TODO: implement collision client

    def get_env_cb(self, goal):
        rospy.loginfo("get env callback running")



        # Fill result msg with data
        self._result.decision_space = [1, 0, 1, 0]

        # Notifies client that action/goal has succeeded
        self.env_server.set_succeeded(self._result)


    def check_for_collision(self):
        pass

    def main(self):
        pass

if __name__ == "__main__":
    rospy.init_node("situational_awareness_py")
    situational_awareness = SituationalAwareness()
    rospy.spin()
