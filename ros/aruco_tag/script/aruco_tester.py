import rospy
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from aruco_msgs.msg import MarkerArray

 # Initialize publisher to control the TurtleBot's movement
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Method to convert the ROS image to an OpenCV image and display
def rosimage_to_cvimage(image_data, bridge):
    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    # Display the image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

# def aruco_display(corners, ids, rejected, image):
# 	if len(corners) > 0:

# 		ids = ids.flatten()

# 		for (markerCorner, markerID) in zip(corners, ids):

# 			corners = markerCorner.reshape((4, 2))
# 			(topLeft, topRight, bottomRight, bottomLeft) = corners

# 			topRight = (int(topRight[0]), int(topRight[1]))
# 			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
# 			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
# 			topLeft = (int(topLeft[0]), int(topLeft[1]))

# 			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
# 			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
# 			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
# 			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

# 			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
# 			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
# 			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

# 			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
# 				0.5, (0, 255, 0), 2)
# 			print("[Inference] ArUco marker ID: {}".format(markerID))

# 	return image

def main():
    # Initialize ROS node
    rospy.init_node('aruco_detection', anonymous=True)
    # Initialize CV bridge to convert ROS images to OpenCV images
    bridge = CvBridge()
    # Initialize subscriber to receive images from the TurtleBot's camera
    image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, rosimage_to_cvimage, bridge)
    # Start the main ROS loop
    rospy.spin()

def shutdown():
    rospy.logininfo("Stopping Turtlebot: - Node Shutdown")
    velocity_publisher.publish(Twist())

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)
    main() 