#! /usr/bin/env python

import rospy
import actionlib

import pathfinder.msg


class EnvServer:
    # define messages for action server
    _feedback = pathfinder.msg.get_envFeedback()
    _result = pathfinder.msg.get_envResult()

    def __init__(self):
        rospy.init_node("env_server")
        self.env_server = actionlib.SimpleActionServer(rospy.get_name(), pathfinder.msg.get_envAction, self.get_env_cb, False)
        self.env_server.start()

    def get_env_cb(self):
        print("get env callback running")

        # Fill result msg with data
        self._result.decision_space = [1, 0, 1, 0]

        # Notifies client that action/goal has succeeded
        self.env_server.set_succeeded(self._result)


if __name__ == "__main__":
    env_server = EnvServer()
    rospy.spin()