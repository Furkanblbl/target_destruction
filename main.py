import cv2
import pyzed.sl as sl
import torch
import time
from pySerialTransfer import pySerialTransfer as txfer
from torchvision.transforms import functional as F

class ROVDataTx: # Data to be sent to the microcontroller
    def __init__(self):
        self.arac_ileri_degeri = 0 # move forward
        self.arac_x_degeri = 0 # move left or right
        self.arac_y_degeri = 0 # move up or down
        self.arac_yengec_degeri = 0 
        self.degree = 0 # rotate

class ROVDataRx: # Data to be received from the microcontroller
    def __init__(self):
        self.RollS = 0 
        self.PitchS = 0
        self.HeadingS = 0
        self.frontLidarS = 0
        self.leftLidarS = 0
        self.rightLidarS = 0
        self.altitudeS = 0
        self.data1S = 0

class ImageProcessor:
    def __init__(self, model_path='bestEftelya.pt'): # load YoLoV5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)# load YoLoV5 model

    def process_frame(self, frame): # image processing function
        with torch.no_grad(): 
            img_tensor = F.to_tensor(frame).unsqueeze(0).cuda() # image to tensor
            result = self.model(img_tensor) # get results from the model
            df = result.pandas().xyxy[0] # convert results to pandas DataFrame

        return df # return DataFrame
    
class ROVController:
    def __init__(self, image_processor, serial_link): # initialize ROVController object
        self.image_processor = image_processor # image_processor object
        self.serial_link = serial_link # serial_link object
