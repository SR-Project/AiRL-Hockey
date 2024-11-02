#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Vector3
from air_hockey.msg import RobotState, RlInfo  # Import your custom messages

# Variables to store the latest data from both topics
puck_position = None
puck_velocity = None
robot_position = None
robot_velocity = None

def puck_state_callback(msg):
    """
    Callback for /puck/state topic. Updates the puck position and velocity.
    """
    global puck_position, puck_velocity
    puck_position = msg.position  # Assuming the puck's position is in geometry_msgs/Point
    puck_velocity = msg.velocity  # Assuming the puck's velocity is in geometry_msgs/Vector3

def robot_state_callback(msg):
    """
    Callback for /robot_state topic. Updates the robot position and velocity.
    """
    global robot_position, robot_velocity
    robot_position = msg.position  # Assuming the robot's position is in geometry_msgs/Point
    robot_velocity = msg.velocity  # Assuming the robot's velocity is in geometry_msgs/Vector3

def main():
    rospy.init_node('rl_info_publisher', anonymous=True)

    # Subscribers for /puck/state and /robot_state
    rospy.Subscriber('/puck/state', RobotState, puck_state_callback)  # Replace with the correct message type for /puck/state
    rospy.Subscriber('/robot_state', RobotState, robot_state_callback)

    # Publisher for the /rl_info topic
    rl_info_pub = rospy.Publisher('/rl_info', RlInfo, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Check if we have data from both topics
        if puck_position is not None and puck_velocity is not None and robot_position is not None and robot_velocity is not None:
            # Create an RlInfo message
            rl_info_msg = RlInfo()
            rl_info_msg.puck_position = puck_position
            rl_info_msg.puck_velocity = puck_velocity
            rl_info_msg.robot_position = robot_position
            rl_info_msg.robot_velocity = robot_velocity

            # Publish the combined data
            rl_info_pub.publish(rl_info_msg)

            # Log the data being published
            '''rospy.loginfo("Publishing rl_info: puck_position(x=%f, y=%f, z=%f), puck_velocity(vx=%f, vy=%f, vz=%f), "
                          "robot_position(x=%f, y=%f, z=%f), robot_velocity(vx=%f, vy=%f, vz=%f)",
                          puck_position.x, puck_position.y, puck_position.z,
                          puck_velocity.x, puck_velocity.y, puck_velocity.z,
                          robot_position.x, robot_position.y, robot_position.z,
                          robot_velocity.x, robot_velocity.y, robot_velocity.z)'''

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
