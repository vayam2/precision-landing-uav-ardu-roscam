#!/usr/bin/env python3
# Imports

import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Gimbal
import time
import math
from pymavlink import mavutil
######################################################################################################################################################
# time.sleep(12)
class ArUcoDetector:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters) # declare a detector with the selected params and dict
        self.vehicle = connect('127.0.0.1:6969')  # Replace with your connection string
        self.arm_and_takeoff(8) 
        self.send_velocity(2, 0, 0)
        print("sending initial velocity")
        self.flag = 1  # Replace 10 with your desired altitude

    def capture_video(self):
        cap = cv2.VideoCapture(0)  # 0 is the default camera index
        while cap.isOpened():
            self.vehicle.gimbal.rotate(0, 0, -90)
            #self.vehicle.gimbal.rotate(0, 0, 90)
            ret, frame = cap.read()
            if ret:
                self.process_frame(frame)
                #cv2.imshow("ArUco Detection", frame)
            #if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()

    def process_frame(self, frame):
        height, width, _ = frame.shape  # Get frame size
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        
        if ids is None:
            self.flag = 1

        if ids is not None:
            # Draw detected markers and calculate center points
            for i in range(len(ids)):
                marker_id = ids[i]
                if marker_id == 4:  
                    marker_corners = corners[i][0]

                    # Draw marker
                    #cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    # Calculate center point
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    center_point = (center_x, center_y)
                    
                    center_frame_x = 320
                    center_frame_y = 240
                    displacement_x = center_x - center_frame_x
                    displacement_y = center_frame_y - center_y
                    displacement = (displacement_x,displacement_y)
                    print(displacement)

                    flag_x = 0
                    flag_y = 0
                    p_x_coeff = 0.0025 #+ (self.vehicle.location.global_relative_frame.alt*0.0001)
                    p_y_coeff = 0.003 #+ (self.vehicle.location.global_relative_frame.alt*0.0001)
                    
                    if displacement_x >= 3 or displacement_x <= -3:
                        v_x = p_x_coeff*displacement_x
                    else:
                        v_x = 0
                        flag_x = 1

                    if displacement_y >= 3 or displacement_y <= -3:
                        v_y = p_y_coeff*displacement_y
                    else:
                        v_y = 0
                        flag_y = 1

                    if flag_x == 1 and flag_y == 1:
                        current_altitude = self.vehicle.location.global_relative_frame.alt
                        land_alti = 1
                        if current_altitude > land_alti:
                            print("Alti when going down")
                            self.send_velocity(0, 0, 1)
                        else:
                            print("mode land")
                            self.vehicle.mode = VehicleMode("LAND")
                    else:
                        self.send_velocity(v_y, v_x, 0)

                    # Draw center point
                    #cv2.circle(frame, center_point, 5, (0, 255, 0), -1)

                    # Display marker ID near the center point
                    #cv2.putText(frame, str(marker_id), center_point, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        else:
            if self.vehicle.location.global_relative_frame.alt >= 8 and self.flag == 1:
                self.send_velocity(2, 0, 0)
                print("Velocity when aruco not detected")
            pass

        #cv2.imshow("ArUco Detection", frame)
        #cv2.waitKey(1)

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
        detector = ArUcoDetector()
        detector.capture_video()
    except KeyboardInterrupt:
        pass

######################################################################################################################################################
