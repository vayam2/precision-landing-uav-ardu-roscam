import cv2

vid_file = 'output.avi'
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

vid = cv2.VideoCapture(0)
# vid = cv2.VideoCapture(vid_file)

while vid.isOpened():
    ret, frame = vid.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)    # converting RGB frame to GrayScale frame
        corners, ids, rejected_pts = detector.detectMarkers(gray)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("Strike Cam", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break