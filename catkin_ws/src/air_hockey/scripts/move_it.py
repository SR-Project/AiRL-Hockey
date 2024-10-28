#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
from tf2_geometry_msgs import do_transform_point

def move_to_world_coords(world_coords):
    rospy.init_node('move_to_world_coords')

    # Initialize MoveIt commander
    robot_arm = MoveGroupCommander('manipulator')  # Adjust the group name according to your robot

    robot_arm.set_named_target("Start")
    robot_arm.go(wait=True)

    # Get the current pose of the robot (which includes orientation)
    current_pose = robot_arm.get_current_pose().pose

    # Extract the current orientation (quaternion) from the robot's pose
    current_orientation = current_pose.orientation

    # Set target position in base_link coordinates while keeping the current orientation
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = world_coords[0]  # X-coordinate
    target_pose.position.y = world_coords[1]  # Y-coordinate
    target_pose.position.z = world_coords[2]  # Z-coordinate

    # Maintain the robot's current orientation
    target_pose.orientation.x = current_orientation.x
    target_pose.orientation.y = current_orientation.y
    target_pose.orientation.z = current_orientation.z
    target_pose.orientation.w = current_orientation.w

    # Move robot to the target position with the current orientation
    robot_arm.set_pose_target(target_pose)
    robot_arm.go(wait=True)

    # Clear the target pose after moving
    robot_arm.clear_pose_targets()

    # Get and print the current joint values after reaching the target position
    joint_values = robot_arm.get_current_joint_values()
    print("Current joint values:", joint_values)
    
if __name__ == "__main__":
    # Example world coordinates (x, y, z)
    world_x = -0.5  # LIMIT 0.6 with z=1.2 and y=0.0
    world_y = 0
    world_z = 0.775

    base_x = -1.2
    base_y = 0.0
    base_z = 0.8

    world_coordinates = [world_x - base_x, world_y - base_y, world_z - base_z]

    print("World Position:", world_x, world_y, world_z)
    print("Relative Robot Position:", world_coordinates)

    move_to_world_coords(world_coordinates)
