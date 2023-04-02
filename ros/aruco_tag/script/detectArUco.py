import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math

class ArUcoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_size = 0.05  # size of the ArUco tag in meters
        self.tag_detected = False

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, rejected_image_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            self.tag_detected = True
            for i, id in enumerate(ids):
                if id == 0:  # move forward
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = 0.1
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.sleep(5)  # move for 10 seconds
                    cmd_vel_msg.linear.x = 0.0  # stop moving
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                elif id == 1:  # turn left
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.angular.z = 0.5
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.sleep(math.pi)  # move for 10 seconds
                    cmd_vel_msg.linear.x = 0.0  # stop moving
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                elif id == 2:  # turn right
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.angular.z = -0.5
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.sleep(math.pi)  # move for 10 seconds
                    cmd_vel_msg.linear.x = 0.0  # stop moving
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                elif id == 3:  # move backward
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = -0.1
                    self.cmd_vel_pub.publish(cmd_vel_msg)
                    rospy.sleep(5)  # move for 10 seconds
                    cmd_vel_msg.linear.x = 0.0  # stop moving
                    self.cmd_vel_pub.publish(cmd_vel_msg)
        else:
            if not self.tag_detected:  # no tag detected yet, move randomly
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.2
                cmd_vel_msg.angular.z = np.random.uniform(-1, 1)
                self.cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':
	try:
		ArUcoDetector()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Exception thrown")