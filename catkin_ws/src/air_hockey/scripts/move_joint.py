#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import RobotCommander, MoveGroupCommander
from numpy.linalg import pinv  # Pseudo-inverse
from air_hockey.msg import RlInfo
from ai import move
import time


VELOCITY_FACTOR = 0.5


class VelocityControlNode:
    def __init__(self):
        rospy.init_node('velocity_control_node', anonymous=True)

        # MoveIt interface
        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander("manipulator")

        # Subscriber to the velocity topic
        rospy.Subscriber('/rl_info', RlInfo, self.velocity_callback)
        
        # Publisher for the joint trajectory controller
        self.pub = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=1)

        # Initialize target velocities
        self.target_velocity = [0.0, 0.0]
        # self.previous_velocity = [0.0, 0.0]
        self.last_command_time = time.time()  # Track last received command time

        # Threshold for minimal velocity command
        self.velocity_threshold = 1e-3  # Adjust as needed

    def velocity_callback(self, data):
        
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
        rl_action = np.array(values, dtype=np.float64)  # Replace with actual RL model output

        rl_action *= VELOCITY_FACTOR

        action_x, action_y = rl_action

        # frame transformation (negate x and reverse)
        action_x = action_x * -1 
        tmp = action_x
        action_x = action_y
        action_y = tmp

        self.last_command_time = time.time()  # Update the time of last received command
        self.target_velocity = [action_x, action_y]

        # # Smoothing factor
        # alpha = 0.05
        # self.target_velocity = alpha * np.array(self.target_velocity) + (1 - alpha) * np.array(self.previous_velocity)
        # self.previous_velocity = self.target_velocity
        # print(self.target_velocity)

    def send_velocity_command(self):
        # Check if idle for too long, reset velocities if necessary
        idle_duration = 1.0  # Time in seconds after which to reset if no command is received
        if time.time() - self.last_command_time > idle_duration:
            self.target_velocity = [0.0, 0.0]

        # Only send command if velocity is above threshold
        if np.linalg.norm(self.target_velocity) < self.velocity_threshold:
            return


        # Solve inverse kinematics for the given velocity to determine joint velocities
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.arm_group.get_active_joints()

        # Use current joint states
        current_joint_positions = self.arm_group.get_current_joint_values()

        # Define the trajectory point based on the desired velocity
        point = JointTrajectoryPoint()
        point.positions = current_joint_positions

        # Here you need to calculate the joint velocities based on IK or your controller
        joint_velocities = self.calculate_joint_velocities(self.target_velocity)
        point.velocities = joint_velocities
        point.time_from_start = rospy.Duration(0.1)  # Small time step for smooth control

        joint_trajectory_msg.points = [point]

        # Publish the joint trajectory to the controller
        self.pub.publish(joint_trajectory_msg)
    

    def calculate_joint_velocities(self, end_effector_velocity):
        """
        Calculates the joint velocities required to achieve a given end-effector velocity.
        :param end_effector_velocity: [vx, vy] linear velocity in Cartesian space
        :return: joint velocities
        """

        # Get the current joint values (positions) of the arm
        current_joint_positions = self.arm_group.get_current_joint_values()
        jacobian = self.arm_group.get_jacobian_matrix(current_joint_positions)
    
        desired_velocity = np.array([end_effector_velocity[0], end_effector_velocity[1], 0, 0, 0, 0])  # vx, vy, vz, wx, wy, wz
        jacobian_pseudo_inverse = pinv(jacobian)
        joint_velocities = np.dot(jacobian_pseudo_inverse, desired_velocity)

        return joint_velocities

    def run(self):
        rate = rospy.Rate(100)  # 10 Hz control loop
        while not rospy.is_shutdown():
            self.send_velocity_command()
            rate.sleep()

if __name__ == '__main__':
    controller = VelocityControlNode()
    controller.run()
