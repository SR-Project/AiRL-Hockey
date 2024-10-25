#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Point
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs  # Import for transforming PoseStamped
import sys

# Initialize moveit_commander and rospy
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('end_effector_position_fk', anonymous=True)

# Initialize robot and group
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("manipulator")  # Replace "arm" with your robot's move group

# TF2 buffer and listener to transform between frames
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Publisher to publish the end-effector position
position_pub = rospy.Publisher('/end_effector_position_base_link', Point, queue_size=10)

rate = rospy.Rate(10)  # 10 Hz

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

        print(current_pose.position.x, current_pose.position.y, current_pose.position.z)


        # Transform the pose from base_link frame to world
        pose_base_link = tf_buffer.transform(pose_world, "world", rospy.Duration(1.0))

        # Extract the position of the end-effector in base_link frame
        end_effector_position = Point()
        end_effector_position.x = pose_base_link.pose.position.x
        end_effector_position.y = pose_base_link.pose.position.y
        end_effector_position.z = pose_base_link.pose.position.z

        # Publish the position to the topic
        position_pub.publish(end_effector_position)

        # Log the position in base_link frame
        rospy.loginfo("End-effector position wrt world: x=%f, y=%f, z=%f",
                      end_effector_position.x, end_effector_position.y, end_effector_position.z)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Could not transform pose from world to base_link frame")

    rate.sleep()

# Shutdown MoveIt! after use
moveit_commander.roscpp_shutdown()
