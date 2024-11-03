#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import tf2_geometry_msgs
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.srv import GetModelState
from air_hockey.msg import PuckState



class PuckDetector(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # Create publishers for position and velocity
        self.state_pub = rospy.Publisher("/puck/state", PuckState, queue_size=10)

        # Buffer to store position and time history
        self.position_history = []
        self.time_history = []
        self.window_size = 5  # Number of frames to keep for velocity calculation
        self.last_valid_position = None
    
    def camera_info_callback(self, data):
        # Save the camera matrix and distortion coefficients from the camera_info topic
        self.camera_matrix = np.array(data.K).reshape(3, 3)
        self.dist_coeffs = np.array(data.D)

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            #cv2.imshow('frame from camera', cv_image)
        except CvBridgeError as e:
            print(e)

        puck_center = self.detect_puck(cv_image)
        #print(puck_center)

        if puck_center is not None:
            # Convert 2D pixel to 3D coordinates
            puck_3d = self.pixel_to_3d(puck_center)
            #print(puck_3d)

            world_coord = self.transform_point(puck_3d)
            #print(f"Coords (x,y,z): {world_coord}")
            #print("Real coords: (0.000026, -0.000011, 0.774995)")

            self.last_valid_position = world_coord

            model_name = "hockey_puck"

            position, orientation = self.get_model_coordinates(model_name)

            '''if position and orientation:
                print(f"x={float(position.x)}, y={float(position.y)}, z={float(position.z)}")
            else:
                print(f"Failed to get the state of the model '{model_name}'")'''

            world_coord = (float(position.x), float(position.y), float(position.z))


            current_time = rospy.get_time()

            self.update_history(world_coord, current_time)

            if len(self.position_history) > 1:
                velocity = self.compute_velocity_weighted()
                #print(f"Velocity (x,y): {velocity}")
            else:
                velocity = (0,0)
            #print(f"Velocity (x,y): {velocity}")
            # Publish the puck's state (position + velocity)
            self.publish_state(world_coord, velocity)

        else:
            # Puck not detected, return prev values
            if self.last_valid_position is not None:
                current_time = rospy.get_time()
                self.update_history(self.last_valid_position, current_time)

                if len(self.position_history) > 1:
                    velocity = self.compute_velocity_weighted()
                    #print(f"Velocity (x,y): {velocity}")
                else:
                    velocity = (0,0)

                # Publish the last known state
                self.publish_state(self.last_valid_position, velocity)

    def publish_state(self, position, velocity):
        """Publish the puck's position and velocity on the /puck/state topic."""
        puck_state_msg = PuckState()

        # Set position
        puck_state_msg.position.x = position[0]
        puck_state_msg.position.y = position[1]
        puck_state_msg.position.z = position[2]

        # Set velocity
        puck_state_msg.velocity.x = velocity[0]
        puck_state_msg.velocity.y = velocity[1]
        puck_state_msg.velocity.z = 0  # Velocity in z-direction is assumed to be zero

        self.state_pub.publish(puck_state_msg)

    def update_history(self, position, current_time):
        """Update the history buffer with new position and time."""
        self.position_history.append(position)
        self.time_history.append(current_time)

        # Keep the buffer size within the window size
        if len(self.position_history) > self.window_size:
            self.position_history.pop(0)
            self.time_history.pop(0)
    
    
    def compute_velocity_weighted(self):
        """Compute weighted average velocity over frames."""
        if len(self.position_history) < 2:
            return (0, 0)  # Need at least 2 frames for velocity calculation

        total_weight = 0
        weighted_vx_sum = 0
        weighted_vy_sum = 0

        # Weights (more recent frames get higher weights)
        weights = np.linspace(1, len(self.position_history), len(self.position_history))

        for i in range(1, len(self.position_history)):
            dx = self.position_history[i][0] - self.position_history[i-1][0]
            dy = self.position_history[i][1] - self.position_history[i-1][1]

            dt = self.time_history[i] - self.time_history[i-1]
            if dt == 0:
                continue  # Avoid division by zero

            vx = dx / dt
            vy = dy / dt

            # Weight the velocity
            weighted_vx_sum += weights[i-1] * vx
            weighted_vy_sum += weights[i-1] * vy
            total_weight += weights[i-1]

        if total_weight == 0:
            return (0, 0)  # Avoid division by zero

        # Compute the weighted average velocity
        vx_avg = weighted_vx_sum / total_weight
        vy_avg = weighted_vy_sum / total_weight

        return (vx_avg, vy_avg)

    def get_model_coordinates(self, model_name):
        # Wait until the service /gazebo/get_model_state is available
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            # Create a service proxy for /gazebo/get_model_state
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            
            # Call the service and pass the model name to get the model state
            model_state = get_model_state(model_name, "world")  # 'world' is the reference frame
            
            if model_state.success:
                position = model_state.pose.position
                orientation = model_state.pose.orientation
                return position, orientation
            else:
                rospy.logwarn("Model not found!")
                return None, None

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None, None

    def detect_puck(self, frame):
        """Detect the puck using color thresholding (assuming blue puck)"""
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color in HSV
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours of the thresholded image
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # draw contours
        #contour_image = cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

        #cv2.imshow('contour', contour_image)

        if contours:
            # Find the largest contour (assuming it's the puck)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the center of the puck from the contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return (cX, cY)
        
        return None
    
    def pixel_to_3d(self, puck_center):
        # Convert 2D image coordinates to 3D camera frame coordinates using pinhole camera model
        if self.camera_matrix is None:
            rospy.logwarn("No camera matrix found.")
            return None
        
        # with our camera x = z, y = -x, z = y
        
        u, v = puck_center
        z = (1.395 - 0.775)
        x = (u - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
        y = (v - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 1]

        #print("From center:",x,y,z)

        x_cam = z
        y_cam = -x
        z_cam = y
        
        return np.array([x_cam, y_cam, z_cam])
    
    def transform_point(self, coords):

        # Create a TF buffer and listener
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # Define the point in the camera frame
        point_camera = geometry_msgs.msg.PointStamped()
        point_camera.header.frame_id = 'camera_link'
        point_camera.point.x = coords[0]
        point_camera.point.y = coords[1]
        point_camera.point.z = coords[2]

        try:
            # Lookup the transformation from camera_link to world
            trans = tf_buffer.lookup_transform('world', 'camera_link', rospy.Time(0), rospy.Duration(1.0))

            # Transform the point
            point_base = tf2_geometry_msgs.do_transform_point(point_camera, trans)
            #return point_base.point.x, point_base.point.y, point_base.point.z
            return point_base.point.x, point_base.point.y, point_base.point.z

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Could not find the transformation')
            return None

if __name__ == '__main__':
    
    puck_detector = PuckDetector()
    rospy.init_node('puck_detector_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()