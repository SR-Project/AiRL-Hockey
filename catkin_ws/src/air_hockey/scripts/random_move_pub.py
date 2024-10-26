#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray

def generate_circle_pair(angle, radius=0.2):
    # Generate the x and y values for circular motion
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    return [x, y]

def main():
    # Initialize the ROS node
    rospy.init_node('circular_movement_publisher', anonymous=True)
    
    # Create a publisher on the /robot_movement topic with Float32MultiArray messages
    pub = rospy.Publisher('/robot_movement', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # Frequency of 10 Hz (10 messages per second)
    
    # Initial angle in radians
    angle = 0.0
    step_size = 0.1  # Step size for angle increments
    
    rospy.loginfo("Starting to publish circular movement data...")

    # Main loop
    while not rospy.is_shutdown():
        # Generate a new pair of values representing circular motion
        new_pair = generate_circle_pair(angle)
        
        # Prepare the message
        msg = Float32MultiArray(data=new_pair)
        
        # Publish the message
        pub.publish(msg)
        
        # Log the values for debugging
        rospy.loginfo("Published values: %s", new_pair)
        
        # Increment the angle for the next iteration (mod 2*pi to keep it in range)
        angle += step_size
        if angle > 2 * math.pi:
            angle -= 2 * math.pi
        
        # Sleep for 1/10th of a second (10 Hz)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
