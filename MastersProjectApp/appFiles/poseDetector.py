#My own class wrapping functionality of googles mediapipe library
#https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

from ultralytics import YOLO
import ultralytics
import cv2 as cv
import numpy as np

import os 
import sys 
currentDir = os.path.dirname(os.path.abspath(sys.argv[0])) 

poseIndexToPart = { 0:"Nose", 1:"Left Eye", 2:"Right Eye", 3:"Left Ear", 4:"Right Ear", 5:"Left Shoulder", 6:"Right Shoulder",
                    7:"Left Elbow", 8:"Right Elbow", 9:"Left Wrist", 10:"Right Wrist", 11:"Left Hip", 12:"Right Hip", 13:"Left Knee",
                    14:"Right Knee", 15:"Left Ankle", 16:"Right Ankle"
}
posePartToIndex = {v: k for k, v in poseIndexToPart.items()}

class PoseDetector():
    #Static variables
    #model locations
    modelDir = 'appFiles/Models/yolo11n-pose.pt'

    #Constructor / Intialiser
    def __init__(self):
        #create pose detection instance
        self.detector = YOLO(os.path.join(currentDir, PoseDetector.modelDir))

    #Run detection on opencv obtained image (default bgr)
    def detect(self, img : np.ndarray):
        results = self.detector(img, verbose = False, conf = 0.5)[0]
        annImage = results.plot()
        return results, annImage
           
    #Extract results of passed part
    def getPartResultByKey(self, key : str, results, id : int = 0):
        #Catch error if part doesn't exist
        try:
            index = posePartToIndex[key]
            landmarksList = results.keypoints.xyn.numpy()[id]
            print(np.array([landmarksList[index][0], landmarksList[index][1]]))
            return np.array([landmarksList[index][0], landmarksList[index][1]])
        except Exception as e:
            # print(f"Requested part '{key}' does not exist - {e}")
            return None
    

#Create "Singleton" instance for importing
detector = PoseDetector()

if __name__ == '__main__':
    #Error will form if model paths are not adjusted
    cam = cv.VideoCapture(0, cv.CAP_DSHOW)
    cam.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

    cv.namedWindow("camera", cv.WINDOW_NORMAL)
    while True:
        ret,img = cam.read()
        res, img = detector.detectAndDraw(img)
        print(detector.getPartResultByKey('nose', res))
        cv.imshow("camera", img)
        cv.waitKey(10)