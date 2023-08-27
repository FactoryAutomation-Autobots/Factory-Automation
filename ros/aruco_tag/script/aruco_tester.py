import rospy
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import int16

 # Initialize publisher to control the TurtleBot's movement
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# Initizlize publisher to publish the IDs of the current Arucos that are detected
# The queue size may actually matter here since there is a capacity for detecting multiple arucos
aruco_publisher = rospy.Publisher('/aruco_detections', int16, queue_size=10)

# Method to convert the ROS image to an OpenCV image and display
def rosimage_to_cvimage(image_data, bridge):
    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    # Display the image
    cv2.imshow("Image window", cv_image)
    detect_and_publish(cv_image)
    cv2.waitKey(3)
    
# this is a helper method that runs the detection code that I found in your detectArUco.py code and then publishes the id to the '/aruco_detections'
# topic which I modified movement_test.py to subscribe to. This is the publication end of the connection of those two ROS nodes.
def detect_and_publish(cv_image)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    corners, ids, rejected_image_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # for now I am just going to process the ids and publish each of them to the topic... 
    # otherwise it is a relatively simple thing to make it return the entire marker array if we start using that
    for i, id in enumerate(ids):
        # the index is not important
        aruco_publisher.publish(id)

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
