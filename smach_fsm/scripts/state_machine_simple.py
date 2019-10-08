#!/usr/bin/env python

import rospy
import smach
#from smach import State


# Define state Foo
class Foo(smach.State):
    def __init__(self):
        
        # Specifying possible outcomes from init state
        smach.State.__init__(self,outcomes=['outcome1','outcome2'])
        self.counter=0

    def execute(self,userdata):
        # Logging messages to 'rosout'
        #rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

# Define state Bar
class Bar(smach.State):
    def __init__(self):
        
        # Specifying possible outcomes from init state
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self,userdata):
        # Logging messages to 'rosout'
        #rospy.loginfo('Executing state Bar')
        return 'outcome2'


# main
def main():
    rospy.init_node('smach_example_state_machine')
    
    # create a SMACH state machine
    # 'outcome4' and 'outcome5' are the only possible ending states
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                transitions={'outcome1':'BAR',
                             'outcome2':'outcome4'})

        smach.StateMachine.add('BAR', Bar(),
                transitions={'outcome2':'FOO'})

        # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()
