import cv2
import numpy as np


class Camera:

    def __init__(self, sensor_id, width=640, height=360, flip_method=0):
        self.sensor_id = sensor_id
        self.width = width
        self.height = height
        self.flip_method = flip_method
        self.pipeline = self.gstreamer_pipeline()
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)


    def gstreamer_pipeline(self):
        return (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! "
            f"video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            f"nvvidconv flip-method={self.flip_method} ! "
            f"video/x-raw, width={self.width}, height={self.height}, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink"

        )
    
    def read(self):
        if not self.cap.isOpened():
            print(f"Camera {self.sensor_id} failed to open.")
            return None
        
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        self.cap.release()