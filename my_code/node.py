#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArUcoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  # Change topic name here
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters()

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        height, width, _ = cv_image.shape  # Get frame size
        print("Frame size: {} x {}".format(width, height))

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            print("Detected ArUco markers: {}".format(ids))
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
                    print (center_point)
                    center_frame_x = 320
                    center_frame_y = 240
                    displacement_x = center_x - center_frame_x
                    displacement_y = center_frame_y - center_y
                    displacement = (displacement_x,displacement_y)
                    print (displacement)
                    # Draw center point
                    cv2.circle(cv_image, center_point, 5, (0, 255, 0), -1)
                    # Display marker ID near the center point
                    cv2.putText(cv_image, str(marker_id), center_point, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        else:
            print("No ArUco markers detected")

        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

