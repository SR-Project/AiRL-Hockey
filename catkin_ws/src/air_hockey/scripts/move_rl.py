#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
import numpy as np
from air_hockey.msg import RlInfo
from ai import move

# Force factor based on your RL agent's scale
FORCE_FACTOR = 0.05  # Adjust this to scale how far the robot moves based on the RL action

# Air hockey table limits for the robot's field (bottom left to center)
WORLD_X_MIN, WORLD_X_MAX = -1.025501, 0.0  # Bottom to Center in X
WORLD_Y_MIN, WORLD_Y_MAX = -0.565701, 0.565701  # Right to Left in Y

BASE_X = -1.25
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
    # Get the current pose of the robot (including orientation)
    current_pose = robot_arm.get_current_pose().pose

    #print(current_pose.position.x, current_pose.position.y, current_pose.position.z)

    # Target position: calculate new position based on relative movement
    target_pose = geometry_msgs.msg.Pose()    
    
    target_pose.position.x = clamp(current_pose.position.x + relative_movement[0], BASE_X_MIN, BASE_X_MAX)
    target_pose.position.y = clamp(current_pose.position.y + relative_movement[1], BASE_Y_MIN, BASE_Y_MAX)
    target_pose.position.z = current_pose.position.z  # Maintain current z-position

    #print(target_pose.position.x, target_pose.position.y, target_pose.position.z)

    # Keep the same orientation
    target_pose.orientation = current_pose.orientation

    # Move the robot to the new position
    robot_arm.set_pose_target(target_pose)
    robot_arm.go(wait=True)

    # Clear target after moving
    robot_arm.clear_pose_targets()

def callback(data):

    #print(data)

    puck_pos = data.puck_position
    puck_vel = data.puck_velocity
    robot_pos = data.robot_position
    robot_vel = data.robot_velocity

    puck_x, puck_y, puck_z = puck_pos.x, puck_pos.y, puck_pos.z
    puck_dx, puck_dy, puck_dz = puck_vel.x, puck_vel.y, puck_vel.z

    robot_x, robot_y, robot_z = robot_pos.x, robot_pos.y, robot_pos.z
    robot_dx, robot_dy, robot_dz = robot_vel.x, robot_vel.y, robot_vel.z

    values = move(puck_x, puck_y, puck_dx, puck_dy, robot_x, robot_y, robot_dx, robot_dy)

        
    # Example RL model action in [-1, 1] for x and y forces
    rl_action = np.array(values)  # Replace with actual RL model output

    #print(rl_action)
    if not (rl_action[0]==0 and rl_action[1]==0):

        # Convert RL action to movement in the world
        relative_movement = rl_action * FORCE_FACTOR  # Scale the action to control movement

        print(relative_movement)

        move_to_world_coords(relative_movement)

if __name__ == "__main__":
    rospy.init_node('move_to_world_coords')

    # Initialize MoveIt commander
    robot_arm = MoveGroupCommander('manipulator')
    
    # Move to start position
    #robot_arm.set_named_target("Start")
    #robot_arm.go(wait=True)

    #current_pose = robot_arm.get_current_pose().pose
    #print(current_pose)
    
    rospy.Subscriber('/rl_info', RlInfo, callback)
    
    rospy.loginfo("Subscribed to /robot_movement topic.")
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()

