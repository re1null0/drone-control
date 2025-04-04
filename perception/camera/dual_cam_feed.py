#!/usr/bin/env python3

import cv2

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0
):
    """
    Return a GStreamer pipeline string for capturing from an IMX219 camera
    on a Jetson using the nvarguscamerasrc element.
    - sensor_id: which CSI camera to use (0 or 1 for a dual setup).
    - capture_width, capture_height: camera capture native resolution.
    - display_width, display_height: desired resolution for output frames.
    - framerate: frames per second.
    - flip_method: 0 means no rotation; can be set 0..7 for rotation/flip.
    """
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "framerate=(fraction)%d/1, format=(string)NV12 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, format=(string)BGRx, "
        "width=(int)%d, height=(int)%d ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def main():
    # Open camera 0 using the pipeline
    cap_left = cv2.VideoCapture(
        gstreamer_pipeline(sensor_id=0), 
        cv2.CAP_GSTREAMER
    )
    
    # Open camera 1 using the pipeline
    cap_right = cv2.VideoCapture(
        gstreamer_pipeline(sensor_id=1), 
        cv2.CAP_GSTREAMER
    )

    # Check if both cameras opened successfully
    if not cap_left.isOpened():
        print("Could not open camera 0")
        return
    if not cap_right.isOpened():
        print("Could not open camera 1")
        return

    print("Successfully opened both cameras. Press 'q' to exit.")

    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        # If frames could not be grabbed, break out
        if not ret_left or not ret_right:
            print("Failed to read from one or both cameras.")
            break

        # Display both frames in separate windows
        cv2.imshow("Camera 0", frame_left)
        cv2.imshow("Camera 1", frame_right)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the captures and close windows
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
