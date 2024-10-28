#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point, Vector3
from air_hockey.msg import RobotState  # Import your custom message
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs  # Import for transforming PoseStamped
import sys
import time

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('end_effector_position_fk', anonymous=True)

rospy.sleep(2)


# Initialize robot and group
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("manipulator")  # Replace "manipulator" with your robot's move group

    
# Move to start position
group.set_named_target("Start")
group.go(wait=True)

# TF2 buffer and listener to transform between frames
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Publisher to publish the RobotState message
state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)

rate = rospy.Rate(10)  # 10 Hz

# Variables to store the previous position and time
previous_position = None
previous_time = None

while not rospy.is_shutdown():

    try:
        # Lookup the transform from base_link to world (inverse of world to base_link)
        transform = tf_buffer.lookup_transform("base_link", "world", rospy.Time(0), rospy.Duration(1.0))
        
        # Get the current pose of the end-effector in the world frame
        current_pose = group.get_current_pose().pose

        # Create a PoseStamped message to transform
        pose_world = geometry_msgs.msg.PoseStamped()
        pose_world.header.frame_id = "base_link"
        pose_world.pose = current_pose

        # Transform the pose from base_link frame to world
        pose_base_link = tf_buffer.transform(pose_world, "world", rospy.Duration(1.0))

        # Extract the position of the end-effector in base_link frame
        end_effector_position = Point()
        end_effector_position.x = pose_base_link.pose.position.x
        end_effector_position.y = pose_base_link.pose.position.y
        end_effector_position.z = pose_base_link.pose.position.z

        # Calculate velocity if we have a previous position and time
        current_time = time.time()
        velocity = Vector3()

        if previous_position is not None and previous_time is not None:
            time_diff = current_time - previous_time
            velocity.x = (end_effector_position.x - previous_position.x) / time_diff
            velocity.y = (end_effector_position.y - previous_position.y) / time_diff
            velocity.z = 0.0  # Assuming no movement in the z direction

        # Create a RobotState message
        robot_state = RobotState()
        robot_state.position = end_effector_position
        robot_state.velocity = velocity

        # Publish the RobotState message
        state_pub.publish(robot_state)

        # Log the RobotState
        #rospy.loginfo("Robot State: Position (x=%f, y=%f, z=%f) Velocity (vx=%f, vy=%f)",
        #              robot_state.position.x, robot_state.position.y, robot_state.position.z,
        #              robot_state.velocity.x, robot_state.velocity.y)

        # Update the previous position and time for the next iteration
        previous_position = end_effector_position
        previous_time = current_time

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Could not transform pose from world to base_link frame")

    rate.sleep()

# Shutdown MoveIt! after use
moveit_commander.roscpp_shutdown()
