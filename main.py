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


    def control_ROV(self): # control ROV function
        rovDataTx = ROVDataTx() # Data to be sent to the microcontroller
        rovDataRx = ROVDataRx() # Data to be received from the microcontroller

        init = sl.InitParameters() # ZED camera initialization
        init.camera_resolution = sl.RESOLUTION.HD720 # Use HD720 video mode (default fps: 60)
        init.depth_mode = sl.DEPTH_MODE.PERFORMANCE # Use PERFORMANCE depth mode

        zed = sl.Camera() # Create a ZED camera object
        if not zed.is_opened(): # Open the camera if it isn't opened
            print("Opening ZED Camera...")
        status = zed.open(init) 
        if status != sl.ERROR_CODE.SUCCESS: # Check that the camera is opened
            print(repr(status))
            exit()
        
        try:
            while True: # Main loop
                runtime_parameters = sl.RuntimeParameters() # Create a RuntimeParameters object for camera parameters
                left_image = sl.Mat() # Create sl.Mat objects for left images
                right_image = sl.Mat() # Create sl.Mat objects for right images

                if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS: # Grab an image from the camera
                    zed.retrieve_image(left_image, sl.VIEW.LEFT) 
                    zed.retrieve_image(right_image, sl.VIEW.RIGHT) 

                    frame = cv2.cvtColor(left_image.get_data(), cv2.COLOR_BGR2RGB) # Convert the image to RGB

                    df_result = self.image_processor.process_frame(frame) # Process the image
                    

                    if df_result is not None and not df_result.empty: # If there is a result from the image processing algorithm 
                        for ind in df_result.index: # For each object detected
                            if df_result['class'][ind] == 0 and df_result['confidence'][ind] >= 0.5: # If the object is a red circle and the confidence is greater than 0.5
                                x1, y1 = int(df_result['xmin'][ind]), int(df_result['ymin'][ind]) # Get the coordinates of the object
                                x2, y2 = int(df_result['xmax'][ind]), int(df_result['ymax'][ind]) # Get the coordinates of the object

                                noktax = int(((x2+x1)/2)) 
                                noktay = int((y2+y1)/2) 
                                nokta = noktax,noktay # get the center of the object
                                
                                alan = (x2-x1) * (y2-y1) # get the area of the object
                                
                                farkx_ = 320 - noktax # get the difference between the center of the object and the center of the image
                                farky_ = 240 - noktay # get the difference between the center of the object and the center of the image
                            
                                label = df_result['name'][ind] # get the label of the object
                                conf =  df_result['confidence'][ind] # get the confidence of the object
                                text = label + " " + str(conf.round(decimals=2)) # get the text to be written on the image
                                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,0),2) # draw a rectangle around the object
                                cv2.putText(frame, text,(x1,y1-5), cv2.FONT_ITALIC,2 ,(0,0,0),2) # write the text on the image
                                cv2.putText(frame, ".", (nokta), cv2.FONT_ITALIC, 1, (0, 0, 255), 4)
                    if alan <= 500 : # if the area of the object is less than 500
                        rovDataTx.arac_x_degeri = 0
                        rovDataTx.arac_ileri_degeri = 150 # move the ROV forward
                
                    if alan >500: # if the area of the object is greater than 500
                        if (-30 < farkx_ < 30):  # if the difference between the center of the object and the center of the image is less than 30 and greater than -30
                            rovDataTx.arac_ileri_degeri = 200 # move the ROV forward

                        else: # if the difference between the center of the object and the center of the image is greater than 30 or less than -30
                            rovDataTx.arac_x_degeri = farkx_ # move the ROV left or right
                            rovDataTx.arac_ileri_degeri = 0
                            
                
                    if 70 > rovDataRx.altitudeS> 65 : # if pressure sensor value is between 65 and 70
                        rovDataTx.arac_ileri_degeri = 0
                        rovDataTx.arac_y_degeri = 150 # move the ROV up
                        
                    if rovDataRx.altitudeS  > 70 : # if pressure sensor value is greater than 70
                        rovDataTx.arac_ileri_degeri = 0
                        rovDataTx.arac_y_degeri = 175 # move the ROV up faster
                        
                    if rovDataRx.altitudeS < 10 : # if pressure sensor value is less than 10
                        rovDataTx.arac_y_degeri = -50 # move the ROV down

                    
                    # send data to the microcontroller
                    sendSize = 0
                    sendSize = self.serial_link.tx_obj(rovDataTx.arac_ileri_degeri, start_pos=sendSize)
                    sendSize = self.serial_link.tx_obj(rovDataTx.arac_y_degeri, start_pos=sendSize)
                    sendSize = self.serial_link.tx_obj(rovDataTx.arac_x_degeri, start_pos=sendSize)
                    sendSize = self.serial_link.tx_obj(rovDataTx.arac_yengec_degeri, start_pos=sendSize)
                    sendSize = self.serial_link.tx_obj(rovDataTx.degree, start_pos=sendSize)
                    self.serial_link.send(sendSize)

                    # check if data is available to read
                    if not self.serial_link.available():
                        if self.serial_link.status < 0:
                            if self.serial_link.status == txfer.CRC_ERROR:
                                print('ERROR: CRC_ERROR')
                            elif self.serial_link.status == txfer.PAYLOAD_ERROR:
                                print('ERROR: PAYLOAD_ERROR')
                            elif self.serial_link.status == txfer.STOP_BYTE_ERROR:
                                print('ERROR: STOP_BYTE_ERROR')
                            else:
                                print('ERROR: {}'.format(self.serial_link.status))
                    

                    # read data from the microcontroller
                    if self.serial_link.available(): 
                        recSize = 0
                        rovDataRx.RollS = self.serial_link.rx_obj(obj_type=type(rovDataRx.RollS), obj_byte_size=sendSize,
                                                                start_pos=recSize) 
                        rovDataRx.PitchS = self.serial_link.rx_obj(obj_type=type(rovDataRx.PitchS),obj_byte_size= sendSize,
                                                                start_pos=recSize + 4)
                        rovDataRx.HeadingS = self.serial_link.rx_obj(obj_type=type(rovDataRx.HeadingS),obj_byte_size= sendSize,
                                                                start_pos=recSize + 8)
                        rovDataRx.frontLidarS = self.serial_link.rx_obj(obj_type=type(rovDataRx.frontLidarS),obj_byte_size= sendSize,
                                                                start_pos=recSize + 12)
                        rovDataRx.leftLidarS = self.serial_link.rx_obj(obj_type=type(rovDataRx.leftLidarS),obj_byte_size= sendSize,
                                                                start_pos=recSize + 16)
                        rovDataRx.rightLidarS = self.serial_link.rx_obj(obj_type=type(rovDataRx.rightLidarS),obj_byte_size= sendSize,
                                                                start_pos=recSize + 20)
                        rovDataRx.altitudeS = self.serial_link.rx_obj(obj_type=type(rovDataRx.altitudeS),obj_byte_size= sendSize,
                                                                start_pos=recSize + 24)
                        rovDataRx.data1S = self.serial_link.rx_obj(obj_type=type(rovDataRx.data1S),obj_byte_size= sendSize,
                                                                start_pos=recSize + 28)

                    if cv2.waitKey(1) & 0xFF == ord('q'): # if q is pressed, exit the program
                        break

        finally: # close the camera
            zed.close()
            cv2.destroyAllWindows()


if __name__ == "__main__": # main function
    serial_link = txfer.SerialTransfer('/dev/ttyACM0') # serial_link object
    serial_link.open() # open the serial link
    time.sleep(2)  # allow some time for the Arduino to completely reset

    try:
        image_processor = ImageProcessor()
        rov_controller = ROVController(image_processor, serial_link)
        rov_controller.control_ROV()

    finally:
        serial_link.close()
