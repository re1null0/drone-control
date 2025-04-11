from flask import Flask, Response
import cv2
from ultralytics import YOLO 

app = Flask(__name__)
camera = cv2.VideoCapture(0)
# model = YOLO("yolov8s.pt")  


def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            print("Failed to read frame")
            break

        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        

# def generate_frames():
#     while True:
#         success, frame = camera.read()
#         if not success:
#             break
        
#         # Add YOLOv8 inference
#         results = model(frame, conf=0.5)  # Adjust confidence threshold
#         annotated_frame = results[0].plot()  # Draw detections on frame
        
#         # Use annotated frame instead of original
#         _, buffer = cv2.imencode('.jpg', annotated_frame)
#         frame_bytes = buffer.tobytes()
        
#         yield (b'--frame\r\n'
#                b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.after_request
def cors(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)