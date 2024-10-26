#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32MultiArray

def generate_random_pair(last_pair, step_size=0.1):
    # Generate a new pair with values close to the previous pair
    new_x = max(-1, min(1, last_pair[0] + random.uniform(-step_size, step_size)))
    new_y = max(-1, min(1, last_pair[1] + random.uniform(-step_size, step_size)))
    return [new_x, new_y]

def main():
    # Initialize the ROS node
    rospy.init_node('random_movement_publisher', anonymous=True)
    
    # Create a publisher on the /robot_movement topic with Float32MultiArray messages
    pub = rospy.Publisher('/robot_movement', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(1)  # Frequency of 1 Hz (1 message per second)
    
    # Initial pair of values
    last_pair = [random.uniform(-1, 1), random.uniform(-1, 1)]

    rospy.loginfo("Starting to publish random movement data...")

    # Main loop
    while not rospy.is_shutdown():
        # Generate a new pair with values close to the previous pair
        new_pair = generate_random_pair(last_pair)
        
        # Prepare the message
        msg = Float32MultiArray(data=new_pair)
        
        # Publish the message
        pub.publish(msg)
        
        # Log the values for debugging
        rospy.loginfo("Published values: %s", new_pair)
        
        # Update last_pair for the next iteration
        last_pair = new_pair
        
        # Sleep for 1 second (1 Hz)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")

