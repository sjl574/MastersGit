import numpy as np
import cv2 as cv
import tensorflow as tf


###    MOVENET IS BROKEN FUCK THIS


lightningParts = ["nose", "left eye", "right eye", "left ear",
        "right ear", "left shoulder", "right shoulder", "left elbow", 
        "right elbow", "left wrist", "right wrist", "left hip", "right hip", 
        "left knee", "right knee", "left ankle", "right ankle"
]

def npToTensor(image : np.ndarray) -> tf.Tensor:
    #resize image to expected input [?x256]
    inputH, inputW,_ = image.shape
    scaleFactor = 256 / inputW
    imgTensor = tf.image.resize_with_pad(np.expand_dims(img, axis=0), int(inputW * scaleFactor), 256)
    imgTensor = tf.cast(imgTensor, dtype = tf.uint8)
    #ret
    return imgTensor


def applylightning(lightning : tf.lite.Interpreter, image : tf.Tensor):
    #Get tensor indices
    tensorIndexIn = lightning.get_input_details()[0]['index']
    tensorIndexOut = lightning.get_output_details()[0]['index']
    #Run model on tensor
    lightning.set_tensor(tensorIndexIn, np.array(image))
    lightning.invoke()
    #Extract results
    modelOut = lightning.get_tensor(tensorIndexOut)[0]
    posesDetected = len(modelOut)
    partDicts = []
    for pose in range(posesDetected):
        posePoints = modelOut[pose]
        posePoints = np.reshape(posePoints, [-1,3])
        partsDict = {}
        for index in len(lightningParts):
            partsDict[lightningParts[index]] = [posePoints[index][1], posePoints[index][0], posePoints[index][3]]
        partDicts.append(partsDict)
    #Now have a list of dicts, easy to sort through, ret
    return partDicts

#Simple drawing of points
def drawKeyPoints(results : list, image : np.ndarray, minConf : float = 0.4 ):
    height, width = image.shape[:2]
    colors = [(255,0,0), (0,255,0), (0,0,255), (125,125,0), (0,125, 125), (125, 0, 125)]
    for poseIndx in len(results):
        color = colors[poseIndx]
        for part in list(results[poseIndx].values()):
            if part[2] > minConf:
                cvPos = (part[1] * height, part[0] * width)
                image = cv.circle(image, cvPos, 4, color, -1)
    return image


model = tf.lite.Interpreter('appFiles/Models/MovenetLightning.tflite')
model.allocate_tensors() ##Allocate memory for model

cam = cv.VideoCapture(0, cv.CAP_DSHOW)
ret, img = cam.read()
cam.release()

tensor = npToTensor(img)
results = applylightning(model, tensor)
annImg = drawKeyPoints(results, img)

cv.imshow("Output", annImg)






# def detect(interpreter, input_tensor):
#   """Runs detection on an input image.

#   Args:
#     interpreter: Interpreter
#     input_tensor: A [1, input_height, input_width, 3] Tensor of type tf.float32.
#       input_size is specified when converting the model to TFLite.

#   Returns:
#     A tensor of shape [1, 6, 56].
#   """

#   input_details = interpreter.get_input_details()
#   output_details = interpreter.get_output_details()

#   is_dynamic_shape_model = input_details[0]['shape_signature'][2] == -1
#   if is_dynamic_shape_model:
#     input_tensor_index = input_details[0]['index']
#     input_shape = input_tensor.shape
#     interpreter.resize_tensor_input(
#         input_tensor_index, input_shape, strict=True)
#   interpreter.allocate_tensors()

#   interpreter.set_tensor(input_details[0]['index'], input_tensor.numpy())

#   interpreter.invoke()

#   keypoints_with_scores = interpreter.get_tensor(output_details[0]['index'])
#   return keypoints_with_scores

# def keep_aspect_ratio_resizer(image, target_size):
#   """Resizes the image.

#   The function resizes the image such that its longer side matches the required
#   target_size while keeping the image aspect ratio. Note that the resizes image
#   is padded such that both height and width are a multiple of 32, which is
#   required by the model.
#   """
#   _, height, width, _ = image.shape
#   if height > width:
#     scale = float(target_size / height)
#     target_height = target_size
#     scaled_width = math.ceil(width * scale)
#     image = tf.image.resize(image, [target_height, scaled_width])
#     target_width = int(math.ceil(scaled_width / 32) * 32)
#   else:
#     scale = float(target_size / width)
#     target_width = target_size
#     scaled_height = math.ceil(height * scale)
#     image = tf.image.resize(image, [scaled_height, target_width])
#     target_height = int(math.ceil(scaled_height / 32) * 32)
#   image = tf.image.pad_to_bounding_box(image, 0, 0, target_height, target_width)
#   return (image,  (target_height, target_width))
