from flask import Flask, Response
import cv2
from ultralytics import YOLO 

app = Flask(__name__)
camera = cv2.VideoCapture(0)
model = YOLO("yolov8n.pt")  
yolo_enabled = False
        

def generate_frames():
    global yolo_enabled

    while True:
        success, frame = camera.read()
        if not success:
            break

        frame = cv2.flip(frame, 0)
        frame = cv2.resize(frame, (640, 480))

        if yolo_enabled:
            results = model(frame, conf=0.5)
            frame = results[0].plot()
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:  # Filter out noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle_yolo', methods=['POST'])
def toggle_yolo():
    global yolo_enabled
    yolo_enabled = not yolo_enabled
    return {'yolo_enabled': yolo_enabled}


@app.after_request
def cors(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)