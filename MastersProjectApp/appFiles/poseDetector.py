#My own class wrapping functionality of googles mediapipe library
#https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2 as cv
import numpy as np

import os 
import sys 
currentDir = os.path.dirname(os.path.abspath(sys.argv[0])) 


class PoseDetector():
    #Static variables
    #model locations
    liteModelDir = 'appFiles/Models/liteModel.task'
    heavyModelDir = 'appFiles/Models/heavyModel.task'

    #Constructor / Intialiser
    def __init__(self, heavyModel : bool = False):
        #create pose detection instance
        modelDir = os.path.join(currentDir, PoseDetector.heavyModelDir) if heavyModel else os.path.join(currentDir, PoseDetector.liteModelDir)
        base_options = python.BaseOptions(model_asset_path=modelDir)
        options = vision.PoseLandmarkerOptions(base_options = base_options, output_segmentation_masks = True)
        self.detector = vision.PoseLandmarker.create_from_options(options)
    
    #Run detection on opencv obtained image (default bgr)
    def detect(self, bgrImg : np.ndarray):
        rgbImg = cv.cvtColor(bgrImg, cv.COLOR_BGR2RGB)
        #convert np image array to mp image format
        mpImg = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgbImg)
        return self.detector.detect(mpImg)
        
    #Just draws on landmarks obtained:https://colab.research.google.com/github/googlesamples/mediapipe/blob/main/examples/pose_landmarker/python/%5BMediaPipe_Python_Tasks%5D_Pose_Landmarker.ipynb#scrollTo=_JVO3rvPD4RN
    def drawLandmarks(self, img : np.ndarray, results) -> np.ndarray:
        landmarksList = results.pose_landmarks
        # Loop through the detected poses to visualize.
        for idx in range(len(landmarksList)):
            landmarks = landmarksList[idx]
            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                img,
                pose_landmarks_proto,
                solutions.pose.POSE_CONNECTIONS,
                solutions.drawing_styles.get_default_pose_landmarks_style()
            )
        return img

    #Run pose detection on image and draw landmarks, returns [drawn on image, results set]
    def detectAndDraw(self, bgrImg : np.ndarray):
        results = self.detect(bgrImg.copy())
        if not results.pose_landmarks:
            return results, bgrImg
        else:  
            return results, self.drawLandmarks(bgrImg, results)

    #Extract the chest positioning info from the results
    def getChestResults(self, results) -> np.ndarray:
        try:
            landmarksList = results.pose_landmarks[0]
        except Exception as e:
            return np.array([0,0])
        leftChest = np.array([landmarksList[11].x, landmarksList[11].y])
        rightChest = np.array([landmarksList[12].x, landmarksList[12].y])
        centerChest = leftChest + ((leftChest-rightChest)/2)
        return centerChest

    
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
        print(detector.getChestResults(res))
        cv.imshow("camera", img)
        cv.waitKey(10)