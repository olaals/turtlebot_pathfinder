#!/usr/bin/env python
import roslib
roslib.load_manifest('followbot')
import rospy, cv2, numpy, sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):

        # CvBridge is an object that converts between OpenCV Images and ROS Image messages.
        self.bridge = CvBridge()
        
        # Create the display window before showing an image creates a resizable 
        # window with content that automatically scales
        #cv2.namedWindow("image window", 1)
        
        # Create a camera subscriber callback
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                        Image, self.image_callback)       

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                        Twist, queue_size=1)

        self.twist = Twist()


    def p_controller(self, err):
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)

    def image_callback(self, msg):
        try:
            # color image with blue-green-red color order 
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # HSV color space HSV better represents how people relate to colors than
        # the RGB color model does.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # The inRange function simply returns a binary mask, where white pixels (255)
        # represent pixels that fall into the upper and lower limit range and black pixels (0) do not.
        lower_yellow = numpy.array([20, 100, 100])
        upper_yellow = numpy.array([30, 255, 255])       
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = cv_image.shape
        search_top = 3*h/4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        """
        Image Moment is a particular weighted average of image pixel intensities, with the help of which we
        can find some specific properties of an image, like radius, area, centroid etc. To find the centroid
        of the image, we generally convert it to binary format and then find its center.

        https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        """        
        
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
            err = cx - w/2
            self.p_controller(err)

        # imshow displays an image in the specified window
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)      

def main(args):
    follower = Follower()
    rospy.init_node('follower')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    # Explicitly close the window when you are done 
    cv2.destroyAllWindoes()


if __name__ == '__main__':
    main(sys.argv)