#!/usr/bin/env python

import roslib
roslib.load_manifest('followbot')
import rospy, cv2, numpy, sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Follower:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                            Image, self.image_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        
        cv2.imshow("Image window", mask)
        cv2.waitKey(3)

def main(args):
    follower = Follower()
    rospy.init_node('follower')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

