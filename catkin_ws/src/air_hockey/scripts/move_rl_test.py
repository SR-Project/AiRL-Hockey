#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
import numpy as np

# Force factor based on your RL agent's scale
FORCE_FACTOR = 0.1  # Adjust this to scale how far the robot moves based on the RL action

# Air hockey table limits for the robot's field (bottom left to center)
WORLD_X_MIN, WORLD_X_MAX = -1.025501, 0.0  # Bottom to Center in X
WORLD_Y_MIN, WORLD_Y_MAX = -0.565701, 0.565701  # Right to Left in Y

BASE_X = -1.6
BASE_Y = 0.0
BASE_Z = 0.8

BASE_X_MIN = WORLD_X_MIN - BASE_X
BASE_Y_MIN = WORLD_Y_MIN - BASE_Y
BASE_X_MAX = WORLD_X_MAX - BASE_X
BASE_Y_MAX = WORLD_Y_MAX - BASE_Y


def clamp(value, min_value, max_value):
    """Clamp the value within the given min and max range."""
    return max(min(value, max_value), min_value)

def move_to_world_coords(relative_movement):
    rospy.init_node('move_to_world_coords')

    # Initialize MoveIt commander
    robot_arm = MoveGroupCommander('manipulator')

    # Get the current pose of the robot (including orientation)
    current_pose = robot_arm.get_current_pose().pose

    # Target position: calculate new position based on relative movement
    target_pose = geometry_msgs.msg.Pose()
    
    
    target_pose.position.x = clamp(current_pose.position.x + relative_movement[0], BASE_X_MIN, BASE_X_MAX)
    target_pose.position.y = clamp(current_pose.position.y + relative_movement[1], BASE_Y_MIN, BASE_Y_MAX)
    target_pose.position.z = current_pose.position.z  # Maintain current z-position

    # Keep the same orientation
    target_pose.orientation = current_pose.orientation

    # Move the robot to the new position
    robot_arm.set_pose_target(target_pose)
    robot_arm.go(wait=True)

    # Clear target after moving
    robot_arm.clear_pose_targets()

if __name__ == "__main__":
    # Example RL model action in [-1, 1] for x and y forces
    rl_action = np.array([0, 1])  # Replace with actual RL model output

    # Convert RL action to movement in the world
    relative_movement = rl_action * FORCE_FACTOR  # Scale the action to control movement

    move_to_world_coords(relative_movement)

