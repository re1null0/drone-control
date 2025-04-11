from ultralytics import YOLO
import cv2

model = YOLO("yolov8s.pt")
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.5)
    annotated = results[0].plot()

    cv2.imshow("YOLO Live", annotated)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
