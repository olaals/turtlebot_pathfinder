#! /usr/bin/env python

import rospy
import actionlib
from smach import State, StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from math import pi

from tutorial_msgs.msg import TurtlebotMoveAction, TurtlebotMoveGoal, TurtlebotMoveResult

def move(turn, forward):
    """
    Returns a TurtleBotMoveGoal with specified turn and forward distance
    """
    goal = TurtlebotMoveGoal()
    goal.turn_distance = turn
    goal.forward_distance = forward

    return goal


if __name__ == "__main__":

    rospy.init_node('mc_tester')
    
    headless_chicken = StateMachine(outcomes=['success', 'failure'])

    with headless_chicken:

        headless_chicken.add('one_step_forward', 
                    SimpleActionState('turtlebot_move', TurtlebotMoveAction, move(0.0, 1.0)),
                    transitions={'succeeded': 'turn_360', 
                                'preempted': 'one_step_forward',
                                'aborted': 'one_step_forward'})

        headless_chicken.add('turn_360',
                    SimpleActionState('turtlebot_move', TurtlebotMoveAction, move(pi, 0.0)),
                    transitions={'succeeded': 'one_step_forward', 
                                'preempted': 'turn_360',
                                'aborted': 'turn_360'})

    # Create and start the introspection server for smach_viewer
    sis = IntrospectionServer(str(rospy.get_name()), headless_chicken, '/SM_ROOT')
    sis.start()

    headless_chicken.execute()
    rospy.spin()

    sis.stop()
