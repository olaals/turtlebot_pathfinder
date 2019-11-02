#! /usr/bin/env python

import rospy
import actionlib
from smach import State, StateMachine

from tutorial_msgs.msg import calculate_pointAction, calculate_pointGoal, go_to_pointAction, go_to_pointGoal


def calc_point():
    
    calc_point_goal = calculate_pointGoal()
    nav_client.send_goal(calc_point_goal)
    nav_client.wait_for_result()

    return nav_client.get_result()


class Traveling(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['success', 'aborted'], 
                        input_keys=['target'], 
                        output_keys=[])

    def execute(self, userdata):
        goal_twist = userdata.target    # TODO: map target to goal_twist
        vehicle_client.send_goal(goal_twist)
        rospy.loginfo("Waiting for vehicle controller...")

        vehicle_client.wait_for_result()
        goal_success = vehicle_client.get_result()

        if goal_success:
            return 'success'
        else:
            return 'aborted'


class Planning(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                        input_keys=[], 
                        output_keys=['target'])

    def execute(self, userdata):
        userdata.target = calc_point()
        return 'success'


if __name__ == "__main__":
    rospy.init_node("mission_control")

    rospy.loginfo("Setting up client for nav_server...")
    nav_client = actionlib.SimpleActionClient("nav_server", calculate_pointAction)
    nav_client.wait_for_server()
    rospy.loginfo("Client for nav_server up and running")
    
    rospy.loginfo("Setting up client for vehicle_server...")
    vehicle_client = actionlib.SimpleActionClient("vehicle_server", go_to_pointAction)
    vehicle_client.wait_for_server()
    rospy.loginfo("Client for vehicle_server up and running")

    # set up state machine
    go_through_maze = StateMachine(outcomes=['success', 'failure'])

    with go_through_maze:

        StateMachine.add('PLANNING', Planning, 
                            transitions={'success': 'TRAVELING'})

        StateMachine.add('TRAVELING', Traveling, 
                            transitions={'success': 'CHECK_IF_THERE_YET', 
                                        'aborted': 'COLLISION_AVOIDANCE'})

        StateMachine.add('CHECK_IF_THERE_YET', None, 
                            transitions={'arrived_at_destination': 'success', 
                                        'not_there_yet': 'PLANNING'})

        StateMachine.add('COLLISION_AVOIDANCE', None, 
                            transitions={'success': 'PLANNING'})
    

    go_through_maze.execute()
