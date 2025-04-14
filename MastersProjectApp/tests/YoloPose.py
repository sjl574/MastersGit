import cv2 as cv
from ultralytics import YOLO
import numpy as np

#Load model from file
model = YOLO("../appFiles/Models/yolo11n-pose.pt")

#Open cam (dshow for faster opening)
cam = cv.VideoCapture(0, cv.CAP_DSHOW)

while True:
    ret, frame = cam.read()
    if not ret:
        break

    #Run model (use .track for tracking instead of just detection)
    results = model(frame, verbose = False)[0]

    #annotate image with results
    annotated_frame = results.plot()

    #Test printout of results
    id = 0
    print(results.keypoints.xy.numpy()[id])

    #Show Results
    cv.imshow("YOLOv11 Pose Detection", annotated_frame)

    # Press 'q' to quit
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    exit()

# Release resources
cam.release()
cv.destroyAllWindows()
