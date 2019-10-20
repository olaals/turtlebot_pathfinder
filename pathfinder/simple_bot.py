#!/usr/bin/env python

import rospy
from math import ceil
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleBot:

    def __init__(self):
        self.range_front = float('inf')
        self.range_left = float('inf')
        self.range_right = float('inf')

        rospy.init_node('simple_bot')
        self.rate = rospy.Rate(10)
    
    def update_dist_callback(self, msg):
        ranges = msg.ranges
 
        self.range_front = ranges[int(len(ranges) / 2)]
        self.range_left = ranges[0]
        self.range_right = ranges[-1]

    def main(self):

        laser_sub = rospy.Subscriber('/scan', LaserScan, self.update_dist_callback)
        vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)

        while not rospy.is_shutdown():
            print(self.range_front)
            if self.range_front < 2.0:
                # Stop
                msg = Twist()
                vel_pub.publish(msg)
            else:
                # GO
                msg = Twist()
                msg.linear.x = 0.3
                if self.range_left < 1:
                    msg.angular.z = -0.3
                elif self.range_right < 1:
                    msg.angular.z = 0.3
                vel_pub.publish(msg)
            self.rate.sleep()



if __name__ == "__main__":
    simple_bot = SimpleBot()
    simple_bot.main()
