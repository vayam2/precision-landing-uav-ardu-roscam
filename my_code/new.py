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
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/roscam/cam/image_raw', Image, self.image_callback)  # Change topic name here
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)  # Replace with your connection string
        self.arm_and_takeoff(10)  # Replace 10 with your desired altitude
        self.send_velocity(1, 0, 0)
        self.flag = 1  # Replace 10 with your desired altitude
    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = cv_image.shape  # Get frame size
        rospy.loginfo("Frame size: {} x {}".format(width, height))

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rospy.loginfo("Detected ArUco markers: {}".format(ids))
            # Draw detected markers and calculate center points
            for i in range(len(ids)):
                marker_id = ids[i]
                if marker_id == 4: 
                    marker_corners = corners[i][0]
                    # Draw marker
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                    # Calculate center point
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    center_point = (center_x, center_y)
                    rospy.loginfo("Center Point: {}".format(center_point))
                    center_frame_x = 320
                    center_frame_y = 240
                    displacement_x = center_x - center_frame_x
                    displacement_y = center_frame_y - center_y
                    displacement = (displacement_x,displacement_y)
                    flag_x = 0
                    flag_y = 1
                    #CODE FOR CONTROL
                    #
                    #
                    #
                    #
                    #
                    #
                    #     
                    if flag_x == 1 and flag_y == 1:
                        print("mode land")
                        self.vehicle.mode = VehicleMode("LAND")   
                    rospy.loginfo("Displacement: {}".format(displacement))
                    # Draw center point
                    cv2.circle(cv_image, center_point, 5, (0, 255, 0), -1)
                    # Display marker ID near the center point
                    cv2.putText(cv_image, str(marker_id), center_point, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    # Send velocity command
                    self.send_velocity(8, 0, 0)  # Send velocity command (8 m/s along the x-axis)

        else:
            self.send_velocity(1, 0, 0)
            rospy.loginfo("No ArUco markers detected")

        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_velocity(self, vx, vy, vz):
        """
        Send velocity command to the vehicle.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # Send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def decrease_altitude(self, decrement):
        """
        Decrease altitude by decrement (meters).
        """
        current_altitude = self.vehicle.location.global_relative_frame.alt
        target_altitude = current_altitude - decrement
        print("Decreasing altitude from {} to {}".format(current_altitude, target_altitude))
        self.vehicle.simple_goto(LocationGlobalRelative(self.vehicle.location.global_relative_frame.lat, self.vehicle.location.global_relative_frame.lon, target_altitude))

if __name__ == '__main__':
    try:
        detector = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass