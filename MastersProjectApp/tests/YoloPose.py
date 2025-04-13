# from ultralytics import YOLO
# import cv2 as cv

# cam = cv.VideoCapture(0)
# ret, image = cam.read()
# cam.release()


# model = YOLO("appFiles/Models/yolo11n-pose.pt")
# results = model.track(image)

# print(results.boxes)

import cv2 as cv
from ultralytics import YOLO

# Load the YOLOv8-pose model
model = YOLO("appFiles/Models/yolo11n-pose.pt")

# Open webcam (0 = default camera)
cam = cv.VideoCapture(0, cv.CAP_DSHOW)

while True:
    ret, frame = cam.read()
    if not ret:
        break

    # Inference
    results = model.track(frame, verbose = False)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()

    # Show the frame
    cv.imshow("YOLOv8 Pose Detection", annotated_frame)

    # Press 'q' to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cam.release()
cam.destroyAllWindows()
