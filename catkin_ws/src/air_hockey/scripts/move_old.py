#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
from tf2_geometry_msgs import do_transform_point

def move_to_world_coords(world_coords):
    rospy.init_node('move_to_world_coords')

    print(world_coords)

    # Initialize MoveIt commander
    robot_arm = MoveGroupCommander('manipulator')  # Adjust the group name according to your robot

    #robot_arm.set_named_target("Start")
    #robot_arm.go(wait=True)

    # Get the current pose of the robot (which includes orientation)
    current_pose = robot_arm.get_current_pose().pose

    # Extract the current orientation (quaternion) from the robot's pose
    current_orientation = current_pose.orientation

    # Wait for the transform between 'world' and 'base_link'
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # Lookup the transform between world and base_link
        transform = tf_buffer.lookup_transform('base_link', 'world', rospy.Time(0), rospy.Duration(5.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Transform lookup between world and base_link failed.")
        return

    print(transform)

    # Create a PointStamped message for world coordinates
    point_world = geometry_msgs.msg.PointStamped()
    point_world.header.frame_id = 'world'
    point_world.point.x = world_coords[0]
    point_world.point.y = world_coords[1]
    point_world.point.z = world_coords[2]

    # Transform the world coordinates to base_link coordinates
    point_base_link = do_transform_point(point_world, transform)

    # Initialize MoveIt commander
    robot_arm = MoveGroupCommander('manipulator')  # Adjust the group name according to your robot
    robot_arm.set_pose_reference_frame('base_link')

    # Set target position in base_link coordinates
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = point_base_link.point.x
    target_pose.position.y = point_base_link.point.y
    target_pose.position.z = point_base_link.point.z

    # Maintain the robot's current orientation
    target_pose.orientation.x = current_orientation.x
    target_pose.orientation.y = current_orientation.y
    target_pose.orientation.z = current_orientation.z
    target_pose.orientation.w = current_orientation.w

    print(target_pose.position.x, target_pose.position.y, target_pose.position.z)

    # Move robot to the target
    robot_arm.set_pose_target(target_pose)
    robot_arm.go(wait=True)

    # Clear the target pose after moving
    robot_arm.clear_pose_targets()

if __name__ == "__main__":


    # Example world coordinates (x, y, z)
    world_coordinates = [0.5, 0.3 ,0.8]
    move_to_world_coords(world_coordinates)