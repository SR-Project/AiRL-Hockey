#!/usr/bin/env python3


import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import RobotCommander, MoveGroupCommander
from numpy.linalg import pinv  # Pseudo-inverse
from air_hockey.msg import RlInfo
from ai import move


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
        rl_action = np.array(values)  # Replace with actual RL model output

        #self.target_velocity = [-rl_action[0], rl_action[1]]
        self.target_velocity = [-1,0]

    def send_velocity_command(self):
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

        # Get the Jacobian matrix at the current joint positions
        jacobian = self.arm_group.get_jacobian_matrix(current_joint_positions)
    
        # Desired end-effector velocity (assuming 2D planar movement, only x, y)
        # In the full 6D space, this would include linear [vx, vy, vz] and angular [wx, wy, wz] velocities
        # Here we're using just the linear part in the x and y directions.
        desired_velocity = np.array([end_effector_velocity[0], end_effector_velocity[1], 0, 0, 0, 0])  # vx, vy, vz, wx, wy, wz

        # Compute the pseudo-inverse of the Jacobian matrix
        jacobian_pseudo_inverse = pinv(jacobian)

        # Calculate the joint velocities required to achieve the desired end-effector velocity
        joint_velocities = np.dot(jacobian_pseudo_inverse, desired_velocity)

        return joint_velocities

    def run(self):
        rate = rospy.Rate(2)  # 10 Hz control loop
        while not rospy.is_shutdown():
            self.send_velocity_command()
            rate.sleep()

if __name__ == '__main__':
    controller = VelocityControlNode()
    controller.run()
