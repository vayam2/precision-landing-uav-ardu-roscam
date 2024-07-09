#!/usr/bin/env python3
# Imports

import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Gimbal
import time
import math
from pymavlink import mavutil
######################################################################################################################################################

class PrecisionLand:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters) # declare a detector with the selected params and dict

        self.lidar_alti = self.vehicle.rangefinder.distance   # unit: metres
        self.land_alti = 1   # unit: metres
        self.takeoff_alti = 10  # unit: metres
        self.flag = 1
        
        self.vehicle = connect('127.0.0.1:6969')  # Replace with your connection string

        self.arm_and_takeoff(8) 
        self.send_velocity(2, 0, 0)
        print("sending initial velocity")

    def captureVideo(self):
        cap = cv2.VideoCapture(0)  # 0 is the default camera index
        while cap.isOpened():
            self.vehicle.gimbal.rotate(-90, 0, 0)
            ret, frame = cap.read()
            if ret:
                self.processFrame(frame)
                cv2.imshow("ArUco Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    
    def processFrame(self, frame):
        height, width, _ = frame.shape  # Get frame size
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)  # Draw marker
            for i in range(len(ids)):
                marker_id = ids[i]
                if marker_id == 4:  
                    marker_corners = corners[i][0]

                    # Calculate center point
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    center_point = (center_x, center_y)

                    cv2.circle(frame, center_point, 5, (0, 255, 0), -1)    # Draw center point                 
                    cv2.putText(frame, str(marker_id), center_point, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)   # Display marker ID near the center point
                    
                    center_frame_x = height / 2
                    center_frame_y = width / 2
                    displacement_x = center_x - center_frame_x
                    displacement_y = center_frame_y - center_y
                    displacement = (displacement_x,displacement_y)
                    print(displacement)
                    
                    flag_x = 0
                    flag_y = 0
                    p_x_coeff = 0.0025
                    p_y_coeff = 0.003

                    self.lidar_alti = self.vehicle.rangefinder.distance

                    desired_pixel_error = self.calcDesiredPixelError()

                    if displacement_x >= desired_pixel_error or displacement_x <= -desired_pixel_error:
                        v_x = p_x_coeff*displacement_x
                    else:
                        v_x = 0
                        flag_x = 1

                    if displacement_y >= desired_pixel_error or displacement_y <= -desired_pixel_error:
                        v_y = p_y_coeff*displacement_y
                    else:
                        v_y = 0
                        flag_y = 1

                    if self.lidar_alti > 3:
                        self.send_velocity(v_x, v_y, 1)
                    elif flag_x == 1 and flag_y == 1: 
                        if self.lidar_alti <= self.land_alti:
                            print("Mode Land")
                            self.vehicle.mode = VehicleMode("LAND")
                        else:
                            print("Alti when going down")
                            self.send_velocity(0, 0, 1)
                    else:
                        self.send_velocity(v_x, v_y, 0)
        
        else:
            if self.lidar_alti >= 8 and self.flag == 1:
                self.send_velocity(2, 0, 0)
                print("Velocity when aruco not detected")
            pass

    def calcDesiredPixelError(self):
        if self.lidar_alti > 4:
            return 10
        elif int(self.lidar_alti) > 2:
            return int(self.lidar_alti)
        return 2

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

######################################################################################################################################################

# Driver Code
if __name__ == '__main__':
    try:
        detector = PrecisionLand()
        detector.captureVideo()
    except KeyboardInterrupt:
        print("Code Terminated Manually")

######################################################################################################################################################