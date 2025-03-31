#My own class wrapping functionality of googles mediapipe library
#https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2 as cv
import numpy as np

#Define parameters here for singleton
heavyModel = False
cameraFovX = 59.34
cameraFovY = 56.5
imageWidth = 1920
imageHeight = 1080


class PoseDetector():
    #Static variables
    #model locations
    liteModelDir = 'Models/liteModel.task'
    heavyModelDir = 'Models/heavyModel.task'
    #Default camera
    defaultFovX = 59.34
    defaultFovY = 56.5
    defaultImageWidth = 1920
    defaultImageHeight = 1080

    #Constructor / Intialiser
    def __init__(self, heavyModel : bool = False, cameraFovX = None, cameraFovY = None, imageWidth = None, imageHeight = None):
        #Set defaults if not passed into constructor      
        self.cameraFovX = PoseDetector.defaultFovX if cameraFovX == None else cameraFovX
        self.cameraFovY = PoseDetector.defaultFovY if cameraFovY == None else cameraFovY
        self.imageWidth = PoseDetector.defaultImageWidth if imageWidth == None else imageWidth
        self.imageHeight = PoseDetector.defaultImageHeight if imageHeight == None else imageHeight
        #create pose detection instance
        modelDir = PoseDetector.liteModelDir if heavyModel else PoseDetector.heavyModelDir
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

    #Extract the central chest position in pixels [x,y] from the results
    def getChestPx(self, results) -> np.ndarray:
        imgSize = np.array([self.imageWidth, self.imageHeight])
        chestNormed = self.getChestResults(results)
        centerChestPx = chestNormed * imgSize
        return centerChestPx
    
    #Extract the angle of the central chest position from the camera axis (xDeg,yDeg) from the results
    def getChestAngle(self, results) -> np.ndarray:
        chestNormed = self.getChestPx(results)
        DPPX = self.cameraFovX / self.imageWidth
        DPPY = self.cameraFovY / self.imageHeight
        xPxFromCenter = chestNormed[0] - (self.imageWidth/2)
        yPxFromCenter = chestNormed[1] - (self.imageHeight/2)
        xDegFromCenter = xPxFromCenter * DPPX
        yDegFromCenter = yPxFromCenter * DPPY
        return np.array([xDegFromCenter, yDegFromCenter])
    
    #Set Image size different to initial values
    def setImageSize(self, imageWidth : int, imageHeight : int) -> None:
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
        return None

#Create "Singleton" instance for importing
detector = PoseDetector(heavyModel, cameraFovX, cameraFovY, imageWidth, imageHeight)

if __name__ == '__main__':
    cam = cv.VideoCapture(0, cv.CAP_DSHOW)
    cam.set(cv.CAP_PROP_FRAME_WIDTH, PoseDetector.defaultImageWidth)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, PoseDetector.defaultImageHeight)

    cv.namedWindow("camera", cv.WINDOW_NORMAL)
    while True:
        ret,img = cam.read()
        res, img = detector.detectAndDraw(img)
        print(detector.getChestAngle(res))
        cv.imshow("camera", img)
        cv.waitKey(10)