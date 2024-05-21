#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

class ArUcoDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_detector', anonymous=True)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('/roscam/cam/image_raw', Image, self.image_callback)  # Change topic name here
        
        # Initialize ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters) # declare a detector with the selected params and dict
        
        # Connect to the vehicle
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)  # Replace with your connection string
        
        # Arm and take off to a certain altitude
        self.arm_and_takeoff(10) 
        self.send_velocity(1, 0, 0)
        print("sending initial velocity")
        
        # Flag to indicate whether to continue sending velocity commands
        self.flag = 1  # Replace 10 with your desired altitude

    def image_callback(self, data):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = cv_image.shape  # Get frame size

        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)

        if ids is not None:
            if self.flag == 1:
                # Stop sending velocity commands if an ArUco marker is detected
                print("stop velocity")
                self.send_velocity(0,0,0)
                self.flag = 0
            
            for i in range(len(ids)):
                marker_id = ids[i]
                if marker_id == 4:  
                    marker_corners = corners[i][0]
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                    
                    # Calculate center point of the marker
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    center_point = (center_x, center_y)
                    
                    # Calculate displacement from the center of the frame
                    center_frame_x = 320
                    center_frame_y = 240
                    displacement_x = center_x - center_frame_x
                    displacement_y = center_frame_y - center_y
                    displacement = (displacement_x,displacement_y)
                    print(displacement)
                    
                    flag_x = 0
                    flag_y = 0
                    p_x_coeff = 0.0025
                    p_y_coeff = 0.0025
                    
                    # Adjust velocity based on displacement
                    if displacement_y >= 5:
                        print("displacement > 10 thus forward")
                        self.send_velocity(p_y_coeff*displacement_y, 0, 0)
                        time.sleep(0.5)
                    elif displacement_y <= -5:
                        print("displacement < 10 thus backward")
                        self.send_velocity(-(p_y_coeff*displacement_y), 0, 0)
                        time.sleep(0.5)
                    else:
                        print("stop velocity")
                        self.send_velocity(0,0,0)
                        flag_x = 1 
                    
                    if displacement_x >= 5:
                        print("displacement < 10 thus right")
                        self.send_velocity(0, p_x_coeff*displacement_x, 0)
                        time.sleep(0.5)
                    elif displacement_x <= -5:
                        print("displacement < 10 thus left")
                        self.send_velocity(0, -(p_x_coeff*displacement_x), 0)
                        time.sleep(0.5)                   
                    else:
                        print("stop velocity")
                        self.send_velocity(0,0,0)
                        flag_y = 1 
   
                    if flag_x == 1 and flag_y == 1:
                        # If marker is centered, land the drone
                        print("mode land")
                        self.vehicle.mode = VehicleMode("LAND")   
                    
                    # Draw center point
                    cv2.circle(cv_image, center_point, 5, (0, 255, 0), -1)
                    # Display marker ID near the center point
                    cv2.putText(cv_image, str(marker_id), center_point, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        else:
            # Continue sending velocity commands if no ArUco markers are detected
            if self.vehicle.location.global_relative_frame.alt >= 9 and self.flag == 1:
                self.send_velocity(1, 0, 0)
                print("no aruco forward velocity for look around")
            
            pass
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    # Remaining methods for vehicle control omitted for brevity...

if __name__ == '__main__':
    try:
        detector = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
