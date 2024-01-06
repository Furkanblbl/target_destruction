import cv2
import pyzed.sl as sl
import torch
import time
from pySerialTransfer import pySerialTransfer as txfer
from torchvision.transforms import functional as F

class ROVDataTx:
    def __init__(self):
        self.arac_ileri_degeri = 0
        self.arac_x_degeri = 0
        self.arac_y_degeri = 0
        self.arac_yengec_degeri = 0
        self.degree = 0

class ROVDataRx:
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
    def __init__(self, model_path='bestEftelya.pt'):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

    def process_frame(self, frame):
        with torch.no_grad():
            img_tensor = F.to_tensor(frame).unsqueeze(0).cuda()
            result = self.model(img_tensor)
            df = result.pandas().xyxy[0]

        return df