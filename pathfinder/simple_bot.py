#!/usr/bin/env python

import rospy
from math import ceil
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleBot:

    def __init__(self):
        self.dist_to_wall_in_front = float('inf')
        rospy.init_node('simple_bot')
        self.rate = rospy.Rate(10)
    
    def update_dist_callback(self, msg):
        ranges = msg.ranges
        mid_index = int(len(ranges) / 2)
        self.dist_to_wall_in_front = ranges[mid_index]

    def main(self):

        laser_sub = rospy.Subscriber('/scan', LaserScan, self.update_dist_callback)
        vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)

        while not rospy.is_shutdown():
            print(self.dist_to_wall_in_front)
            if self.dist_to_wall_in_front < 0.5:
                # Stop
                msg = Twist()
                vel_pub.publish(msg)
            else:
                # GO
                msg = Twist()
                msg.linear.x = 0.3
                vel_pub.publish(msg)
            self.rate.sleep()



if __name__ == "__main__":
    simple_bot = SimpleBot()
    simple_bot.main()
