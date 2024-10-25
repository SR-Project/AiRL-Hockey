#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
from tf2_geometry_msgs import do_transform_point

def move_to_world_coords(world_coords):
    rospy.init_node('move_to_world_coords')

    '''# Wait for the transform between 'world' and 'base_link'
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
    point_base_link = do_transform_point(point_world, transform)'''

    # Initialize MoveIt commander
    robot_arm = MoveGroupCommander('manipulator')  # Adjust the group name according to your robot
    #robot_arm.set_pose_reference_frame('world')

    # Set target position in base_link coordinates
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = world_coords[0]  #point_base_link.point.x
    target_pose.position.y = world_coords[1]  #point_base_link.point.y
    target_pose.position.z = world_coords[2]  #point_base_link.point.z

    #print(point_base_link.point.x, point_base_link.point.y, point_base_link.point.z)

    # Move robot to the target
    robot_arm.set_pose_target(target_pose)
    robot_arm.go(wait=True)

    # Clear the target pose after moving
    robot_arm.clear_pose_targets()

if __name__ == "__main__":
    # Example world coordinates (x, y, z)

    world_x = -0.8 # LIMIT 0.6 with z=1.2 and y=0.0
    world_y = 0.0
    world_z = 1.2

    base_x = -1.6
    base_y = 0.0
    base_z = 0.8

    world_coordinates = [world_x-base_x, world_y-base_y, world_z-base_z]

    print("World:", world_x, world_y, world_z)
    print("Robot", world_coordinates)

    move_to_world_coords(world_coordinates)
