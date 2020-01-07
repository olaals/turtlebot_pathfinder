#! /usr/bin/env python

import rospy
import actionlib
from smach import State, StateMachine, Concurrence
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

from smach_ros import SimpleActionState, ConditionState, MonitorState, IntrospectionServer

from tutorial_msgs.msg import calculate_pointAction, calculate_pointGoal, go_to_pointAction, go_to_pointGoal
from tutorial_msgs.msg import TurtlebotMoveAction, TurtlebotMoveGoal, TurtlebotMoveResult


class Planning(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                        input_keys=[], 
                        output_keys=['target_turn', 'target_forward'])

    def execute(self, userdata):
        userdata.target_turn = 0.2
        userdata.target_forward = 1.0
        return 'success'


class CollisionAvoidance(State):
    """
    This state should to two things: 
    1) prevent iminent crash and 
    2) recover from crashes 

    """
    def __init__(self):
        State.__init__(self, outcomes=['success', 'failure'], 
                        input_keys=[],
                        output_keys=['target_turn', 'target_forward'])

    def execute(self, userdata):
        # TODO: collision avoidance and/or recovery routine
        return 'success'


def collsion_check(userdata, msg):
    return False


def move(turn, forward):
    """
    Returns a TurtleBotMoveGoal with specified turn and forward distance
    """
    goal = TurtlebotMoveGoal()
    goal.turn_distance = turn
    goal.forward_distance = forward

    return goal


if __name__ == "__main__":
    rospy.init_node("mission_control")

    # rospy.loginfo("Setting up client for nav_server...")
    # nav_client = actionlib.SimpleActionClient("nav_server", calculate_pointAction)
    # nav_client.wait_for_server()
    # rospy.loginfo("Client for nav_server up and running")
    
    # rospy.loginfo("Setting up client for vehicle_server...")
    # vehicle_client = actionlib.SimpleActionClient("vehicle_server", go_to_pointAction)
    # vehicle_client.wait_for_server()
    # rospy.loginfo("Client for vehicle_server up and running")


    # set up state machine
    go_through_maze = StateMachine(outcomes=['success', 'failure'])

    # init userdata
    go_through_maze.userdata.target_turn = 0.0
    go_through_maze.userdata.target_forward = 0.0

    with go_through_maze:

        StateMachine.add('PLANNING', Planning(), 
                        transitions={'success': 'TRANSIT'})

    
        transit = Concurrence(outcomes=['at_goal', 'at_target', 'collision_alarm', 'failure'],
                            default_outcome = 'failure',
                            child_termination_cb = (lambda x: True),
                            input_keys=['target_turn', 'target_forward'],
                            outcome_map={'at_goal': {'GOAL_CHECK': 'true'},
                                        'at_target': {'MOVE': 'succeeded'},
                                        'collision_alarm': {'COLLISION_CHECK': 'true'},
                                        'failure': {'MOVE': 'aborted'}})

        with transit:

            Concurrence.add('MOVE', 
                            SimpleActionState('turtlebot_move', TurtlebotMoveAction, 
                                                move(go_through_maze.userdata.target_turn, 
                                                go_through_maze.userdata.target_forward)))
            
            Concurrence.add('COLLISION_CHECK', MonitorState(
                'collision_alarm', # TODO: find topic name
                LaserScan,
                cond_cb=collsion_check,
                max_checks=-1,
                input_keys=[],
                output_keys=[]
            ))

        StateMachine.add('TRANSIT', transit,
                        transitions={'at_goal': 'success',
                                    'at_target': 'PLANNING',
                                    'collision_alarm': 'COLLISION_AVOIDANCE',
                                    'failure': 'failure'})
                        
        StateMachine.add('COLLISION_AVOIDANCE', CollisionAvoidance(), 
                        transitions={'success': 'TRANSIT', 
                                    'failure': 'failure'})
    

    sis = IntrospectionServer('mc_introspection', go_through_maze, '/SM_TOP')
    sis.start()

    go_through_maze.execute()
    sis.stop()
