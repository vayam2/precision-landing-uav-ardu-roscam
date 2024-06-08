# Imports

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

##########################################################################################

# ArUco Tracking

class ArucoTracker:
    
    # constructor
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/roscam/cam/image_raw', Image, self.image_callback)  # Change topic name here
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.tracker = None
        self.tracking_started = False
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)  # Replace with your connection string. Should be passed as an argument in terminal
        self.arm_and_takeoff(10)
        self.send_velocity(1, 0, 0) # Set this initial velocity to (5, 0, 0) for real world
        '''
        for next phase, we will send the dock's geo-coordinates to the UAV
        This geo-coordinate will be used to calculate the direction of initial velocity
        '''
        print("sending initial velocity")
        self.flag = 1

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = cv_image.shape  # Get frame size
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)

        if ids is None:
            self.flag = 1

        else:
            for i in range(len(ids)):
                marker_id = ids[4]

                if marker_id == 4:  # this value will always depend on the arUco marker id of the UAV's docking station. Should be passed as an argument in terminal
                    marker_corners = corners[i][0]
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids, (255, 105, 180)) # drawing PINK bounding box which shows the DETECTION that is being done
                    
                    # Initialise Tracking if not tracking
                    if not self.tracking_started:
                        self.tracking_started = True
                        print("Tracking Started")
                        self.tracker = cv2.TrackerKCF_create()  # create KCF Tracker
                        bbox = cv2.boundingRect(corners[i][0])
                        self.tracker.init(cv_image, bbox)   # initialising tracker with a bounding box

        if self.tracking_started:
            success, bbox = tracker.update(cv_image)

            if success:
                (x, y, w, h) = [int(i) for i in bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)    # creating GREEN colour bounding box for TRACKING

                # Tracker center coordinates
                center_x_aruco = int(np.mean(bbox[:, 0]))
                center_y_aruco = int(np.mean(bbox[:, 1]))
                center_point_aruco = (center_x_aruco, center_y_aruco)

                # Frame desired center
                center_frame_x = 320
                center_frame_y = 240
                '''
                These center frame values must be adjusted when shifted to real world UAV.
                This is because in ROS, the camera is at the centerand facing below, where as
                in our UAV, the camera is at center in x-axis, but it is shifted forward in y-axis
                '''

                # Calculating displacement
                displacement_x = center_x_aruco - center_frame_x
                displacement_y = center_y_aruco - center_frame_y
                displacement = (displacement_x, displacement_y)
                print("displacement : ", displacement)

                flag_x = 0
                flag_y = 0
                # the above mentioned checkwhether we've to send the velocity along the respective axis or not

                # P - controller coefficients
                p_x_coeff = 0.0025
                p_y_coeff = 0.003

                # calculating velocities
                if displacement_x >= 2 or displacement_x <= -2:
                        v_x = p_x_coeff*displacement_x
                    else:
                        v_x = 0
                        flag_x = 1

                    if displacement_y >= 2 or displacement_y <= -2:
                        v_y = p_x_coeff*displacement_y
                    else:
                        v_y = 0
                        flag_y = 1

                # X and Y axes errors corrected, flick mode land
                if flag_x == 1 and flag_y == 1:
                    print("mode land")
                    self.vehicle.mode = VehicleMode("LAND")
                # Send the calculated velocity
                else:
                    self.send_velocity(v_y, v_x, 0)

                cv2.circle(cv_image, center_point, 5, (0, 0, 255), -1)  # Drawing the center point
                cv2.putText(cv_image, str(marker_id), center_point, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)    # Display marker id near the centr point

            else:
                print("Tracking Lost")
                self.tracking_started = False
        
        else:
            if self.vehicle.location.global_relative_frame.alt >= 9 and self.flag == 1:
                self.send_velocity(1, 0, 0) # Set this initial velocity to (5, 0, 0) for real world
                print("Velocity when aruco not detected")

            pass

        cv2.imshow("ArUco Tracking", cv_image)
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

if __name__ == '__main__':
    try:
        detector = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
