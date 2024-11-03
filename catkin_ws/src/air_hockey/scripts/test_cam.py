#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

class LoadImage(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_callback) # subscribe to the camera image topic, if different, change the topic
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding ="bgr8")
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow('frame from camera', cv_image)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for blue color in HSV
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        #cv2.imshow('frame from camera', mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # draw contours
        #contour_image = cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

        
        #cv2.imshow('contour', contour_image)

        if contours:
            # Find the largest contour (assuming it's the puck)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the center of the puck from the contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                #return (cX, cY)
            
            cv2.circle(cv_image, (cX, cY), 2, (0, 0, 255), -1)  # Draw red dot at center
        
        cv2.imshow('contour', cv_image)

        cv2.waitKey(0)

if __name__ == '__main__':
    
    load_image_object = LoadImage()
    rospy.init_node('load_image_node', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()