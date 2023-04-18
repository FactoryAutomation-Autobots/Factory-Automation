import rospy
import cv2
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import int16

# Initialize publisher to control the TurtleBot's movement
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# Creates a subscriber that recieves an int16 (simple message from the std_msgs package that contains one 16-bit int, which is all that is needed)
aruco_detect_subscriber = rospy.Subscriber('/aruco_detections', int16, aruco_detection_callback)
# defines and initializes an empty list that will store the detected tags that have not yet been handled. I assume that we will want to effectively treat it as
# a queue and add and remove things in first-in first-out (FIFO) order.
detected_tags = []

# This is a simple callback that will be called by the rosmaster whenever a new tag is detected and published on the aruco_detections topic. It is saved in the
# detected_tags global set. The intention for this set is that when there is an aruco tag detected it is added to the set. Once that tag has been executed, it will be removed.
# data will be a 16-bit int
def aruco_detection_callback(data):
    detected_tags.add(data)

def process_tags():
    while not rospy.is_shutdown():
        if len(detected_tags) != 0:
            call_movement_function(detected_tags.pop(0))
        # This will check for a new tag 10 times every second (if it is not already processing tags). If it is processing tags, it will take however long
        # the given movement command takes to execute before checking again. This may not be the best implementation if we don't want a queue-style behavior
        rospy.rate.sleep(100)

# helper method that manages the functionality attached to each aruco tag id
def call_movement_function(tag_id):
    if id == 0:  # move forward
        move_forward(1.0, velocity_publisher)
    elif id == 1:  # turn left
        turn_left(1.0, velocity_publisher)
    elif id == 2:  # turn right
        turn_right(1.0, velocity_publisher)
    elif id == 3:  # move backward
        move_backward(1.0, velocity_publisher)
    else
        # informs the user if a tag_id with an unkown (or not yet implemented) action is being processed. Nothing will happen other than the message
        rospy.logInfo('The tag_id does not correspond to a currently supported action! Tag id: %d',%(tag_id))
    
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
    

def shutdown():
    rospy.logininfo("Stopping Turtlebot: - Node Shutdown")
    velocity_publisher.publish(Twist())

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('movement_tester', anonymous=True)
    rospy.on_shutdown(shutdown)
    move_forward(1.0,velocity_publisher)
    # makes the initial call to the main looping method in this inline script, the process_tags method, which runs until the node shuts down
    process_tags()
   
