import cv2

def detect_markers_webcam():
    # Open the webcam (change the index as needed, typically 0 or 1 for built-in webcams)
    cap = cv2.VideoCapture(0)
    
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    
    # Load the predefined dictionary of ArUco markers
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    
    # Initialize the detector parameters
    parameters = cv2.aruco.DetectorParameters_create()
    
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image")
            break
        
        # Convert the frame to grayscale (required by detectMarkers function)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # Draw detected markers on the frame
        if ids is not None and len(ids) > 0:
            frame_with_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            frame_with_markers = frame
        
        # Display the frame with detected markers
        cv2.imshow('Detected Markers', frame_with_markers)
        
        # Check for 'q' key to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Call the function to detect ArUco markers from webcam feed
detect_markers_webcam()

