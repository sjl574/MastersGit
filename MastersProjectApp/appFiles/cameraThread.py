from PyQt5.QtGui import QImage
from PyQt5.QtCore import QThread,pyqtSignal as Signal
import cv2 as cv
import numpy as np
from appFiles.poseDetector import detector


#Class for handling camera operation in alt thread
class ImagingThread(QThread):
    #Static class variables
    frame_signal = Signal(QImage)
    heavyModel = False
    cameraFovX = 59.34
    cameraFovY = 56.5
    imageWidth = 1920
    imageHeight = 1080
    imageWidthResized = 640
    imageHeightResized = 480
    correctionFactor_x = 1.75
    correctionFactor_y = 0.5
    #globally accessed variables
    running = False
    cameraNum = 0

    #set Camera Number
    def setCameraNum(self, cameraNum):
        ImagingThread.cameraNum = cameraNum
        return None


    #Function override (Run thread - this will be run upon class.start())
    def run(self):
        self.results = None
        #connect camera
        self.cam = cv.VideoCapture(ImagingThread.cameraNum, cv.CAP_DSHOW)
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, ImagingThread.imageWidth)
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, ImagingThread.imageHeight)
        #Run camera whilst flag active
        while ImagingThread.running:
            ret, rawFrame = self.cam.read()
            if not ret:
                ImagingThread.running = False
                print(f"Failed to read camera Image! ")
                break
            self.results, frame = detector.detect(rawFrame)
            frame = self.cvToLabel(frame)
            #This emits a signal to the application containing the image
            self.frame_signal.emit(frame)
        #upon exit, disconnect camera
        self.cam.release()
        ImagingThread.running = False
    
    #Convert cv np image to qt label image type
    def cvToLabel(self,image):
        image = cv.resize(image, (self.imageWidthResized, self.imageHeightResized))
        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        image = QImage(image,
                       image.shape[1],
                       image.shape[0],
                       3 * image.shape[1],
                       QImage.Format_RGB888)
        return image
    
    #Obtain raw results from last image (these are normed pixels)
    def getResults(self):
        return self.results
    
    #obtain pixel position of key part via string identifier
    def getPartPixels(self, partKey : str):
        #ensure results exist
        if self.results is None:
            return None
        #Get parts and ensure that part was detected
        normedPx = detector.getPartResultByKey(partKey, self.results)
        if normedPx is None:
            return None
        #Convert norm to image px coordinates
        pixels = self.normedToPx(normedPx)
        if not pixels.all():
            return None
        #ret pixel location
        return pixels
    
    #Obtain pixel position of chest (not a key part, calculated by mid shoulder position)
    def getChestPixels(self):
        #Check results exist
        if self.results is None:
            return None
        #get shoulder px locations
        leftShoulderPx = self.getPartPixels("Left Shoulder")
        rightShoulderPx = self.getPartPixels("Right Shoulder")
        #Check parts exist
        if (leftShoulderPx is None) or (rightShoulderPx is None):
            return None
        #Find midpoint
        chestPx = np.array([(leftShoulderPx[0] + rightShoulderPx[0])/2, (leftShoulderPx[1] + rightShoulderPx[1])/2])
        #Return
        return chestPx

    @classmethod
    def normedToPx(cls, xyNormed) -> np.ndarray:
        imgSize = np.array([cls.imageWidthResized, cls.imageHeightResized])
        centerChestPx = xyNormed * imgSize
        return centerChestPx

    @classmethod
    def pxToAngle(cls, xyPx) -> np.ndarray:
        DPPX = cls.cameraFovX / cls.imageWidthResized
        DPPY = cls.cameraFovY / cls.imageHeightResized
        xPxFromCenter = xyPx[0] - (cls.imageWidthResized/2)
        yPxFromCenter = xyPx[1] - (cls.imageHeightResized/2)
        xDegFromCenter = xPxFromCenter * DPPX *cls.correctionFactor_x
        yDegFromCenter = yPxFromCenter * DPPY *cls.correctionFactor_y
        return np.array([xDegFromCenter, yDegFromCenter])
        
    @classmethod
    def normedToAngle(cls, xyNormed) -> np.ndarray:
        xyPx = cls.normedToPx(xyNormed)
        xyAngles = cls.pxToAngle(xyPx)
        return xyAngles