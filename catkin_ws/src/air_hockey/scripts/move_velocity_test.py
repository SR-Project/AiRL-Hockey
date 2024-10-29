#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import RobotCommander, MoveGroupCommander
from numpy.linalg import pinv  # Pseudo-inverse
import time

class VelocityControlNode:
    def __init__(self):
        rospy.init_node('velocity_control_node', anonymous=True)

        # MoveIt interface
        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander("manipulator")

        # Subscriber to the velocity topic
        rospy.Subscriber('/robot_movement', Float32MultiArray, self.velocity_callback)

        # Publisher for the joint trajectory controller
        self.pub = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=1)

        # Initialize target velocities
        self.target_velocity = [0.0, 0.0]
        self.last_command_time = time.time()  # Track last received command time

        # Threshold for minimal velocity command
        self.velocity_threshold = 1e-3  # Adjust as needed

    def velocity_callback(self, data):
        values = data.data
        if len(values) == 2:
            x, y = values
            rospy.loginfo("Received values: x=%f, y=%f", x, y)
            self.last_command_time = time.time()  # Update the time of last received command
            self.target_velocity = [x, y]
        else:
            rospy.logwarn("Received an unexpected number of values: %s", values)

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

        # Calculate the joint velocities
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
        current_joint_positions = self.arm_group.get_current_joint_values()
        jacobian = self.arm_group.get_jacobian_matrix(current_joint_positions)

        desired_velocity = np.array([end_effector_velocity[0], end_effector_velocity[1], 0, 0, 0, 0])  # vx, vy, vz, wx, wy, wz
        jacobian_pseudo_inverse = pinv(jacobian)
        joint_velocities = np.dot(jacobian_pseudo_inverse, desired_velocity)

        return joint_velocities

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        while not rospy.is_shutdown():
            self.send_velocity_command()
            rate.sleep()

if __name__ == '__main__':
    controller = VelocityControlNode()
    controller.run()

