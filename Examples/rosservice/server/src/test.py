#!/usr/bin/env python

import rospy
#import sys
#print(sys.path)
#sys.path.append('/rosservice/client/')
#print(sys.path)
from tutorial_msgs.srv import ControlMode


if __name__ == '__main__':


	try:
		rospy.init_node('test_node')
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

