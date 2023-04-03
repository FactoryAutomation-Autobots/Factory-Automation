import rospy
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from aruco_msgs.msg import MarkerArray

 # Initialize publisher to control the TurtleBot's movement
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

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

# Method to convert the ROS image to an OpenCV image and display
def rosimage_to_cvimage(image_data, bridge):
    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
    # Display the image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

# Method to detect the aruco markers and call movement methods depending on aruco id
def tag_detection(image_data, velocity_publisher):
    # Loop through all detected tags
    for marker in image_data.markers:
        # Calculate the distance to the tag based on its size in the image
        distance = 1.0 / marker.confidence
        # Check if the tag has the desired ID
        if marker.id == 0:
            # Print the distance to the tag (optional)
            rospy.loginfo("Tag detected at distance: {}".format(distance))
            # Make sure that the ArUco tag being detected is within a specific distance. 
            if distance > 0.5 and distance < 1.5:
                move_forward(1.0, velocity_publisher)
        elif marker.id == 1:
            rospy.loginfo("Tag detected at distance: {}".format(distance))
            if distance > 0.5 and distance < 1.5:
                turn_left(1.57, velocity_publisher)
        elif marker.id == 2:
            rospy.loginfo("Tag detected at distance: {}".format(distance))
            if distance > 0.5 and distance < 1.5:
                turn_right(1.57, velocity_publisher)
        elif marker.id == 3:
            rospy.loginfo("Tag detected at distance: {}".format(distance))
            if distance > 0.5 and distance < 1.5:
                move_backward(1.0, velocity_publisher)
        
def move_forward(distance_to_move, velocity_publisher):
    # Create a Twist message to control the TurtleBot's movement
    velocity_msg = Twist()
    # Set the linear velocity of the TurtleBot based on the distance to the tag
    velocity_msg.linear.x = 0.1 
    # Calculate the time required to move the specified distance
    time = distance_to_move / velocity_msg.linear.x
    # Publish the Twist message to move the TurtleBot
    velocity_publisher.publish(velocity_msg)
    # Sleep for the calculated time to allow the TurtleBot to move the specified distance
    rospy.sleep(time)
    # Stop the TurtleBot after moving the specified distance
    velocity_msg.linear.x = 0.0
    velocity_publisher.publish(velocity_msg)

def turn_left(distance_to_turn, velocity_publisher):
    # Create a Twist message to control the TurtleBot's movement
    velocity_msg = Twist()
    # Set the angular velocity of the TurtleBot to turn left
    velocity_msg.angular.z = math.radians(45)
    # Calculate the time required to move the specified distance
    time = distance_to_turn / velocity_msg.linear.x
    # Publish the Twist message to turn the TurtleBot
    velocity_publisher.publish(velocity_msg)
    # Sleep for the calculated time to allow the TurtleBot to move the specified distance
    rospy.sleep(time)
    # Stop the TurtleBot after moving the specified distance
    velocity_msg.angular.z= 0.0
    velocity_publisher.publish(velocity_msg)

def turn_right(distance_to_turn, velocity_publisher):
    # Create a Twist message to control the TurtleBot's movement
    velocity_msg = Twist()
    # Set the angular velocity of the TurtleBot to turn right
    velocity_msg.angular.z = math.radians(-45)
    # Publish the Twist message to turn the TurtleBot
    velocity_publisher.publish(velocity_msg)
    # Calculate the time required to move the specified distance
    time = distance_to_turn / velocity_msg.linear.x
    # Publish the Twist message to turn the TurtleBot
    velocity_publisher.publish(velocity_msg)
    # Sleep for the calculated time to allow the TurtleBot to move the specified distance
    rospy.sleep(time)
    # Stop the TurtleBot after moving the specified distance
    velocity_msg.angular.z= 0.0
    velocity_publisher.publish(velocity_msg)

def move_backward(distance_to_move, velocity_publisher):
    # Create a Twist message to control the TurtleBot's movement
    velocity_msg = Twist()
    # Set the linear velocity of the TurtleBot based on the distance to the tag
    velocity_msg.linear.x = -0.1 
    # Calculate the time required to move the specified distance
    time = distance_to_move / velocity_msg.linear.x
    # Publish the Twist message to move the TurtleBot
    velocity_publisher.publish(velocity_msg)
    # Sleep for the calculated time to allow the TurtleBot to move the specified distance
    rospy.sleep(time)
    # Stop the TurtleBot after moving the specified distance
    velocity_msg.linear.x = 0.0
    velocity_publisher.publish(velocity_msg)

def main():
    # Initialize ROS node
    rospy.init_node('aruco_detection', anonymous=True)
    # Initialize CV bridge to convert ROS images to OpenCV images
    bridge = CvBridge()
    # Initialize subscriber to receive images from the TurtleBot's camera
    image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, rosimage_to_cvimage, bridge)
    # Initialize subscriber to receive ArUco tag messages
    tag_subscriber = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, tag_detection, velocity_publisher)
    # Start the main ROS loop
    rospy.spin()

def shutdown():
    rospy.logininfo("Stopping Turtlebot: - Node Shutdown")
    velocity_publisher.publish(Twist())

if __name__ == '__main__':
    rospy.on_shutdown(shutdown)
    main()