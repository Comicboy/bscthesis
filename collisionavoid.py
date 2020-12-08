# Importing packages for the neural network
import numpy as np
import tensorflow
from tensorflow.keras import models
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from tensorflow.keras.applications.inception_v3 import preprocess_input

# Importing the packages for image capture
import cv2

# Importing the packages for the jetson controll
import subprocess
import time

def avoid():
  # Processing the image frame returned by the image capture
  ret, frame = cap.read()
  resized = cv2.resize(frame, (299,299), interpolation = cv2.INTER_AREA)
  image = img_to_array(resized)
  image = image.reshape((1, image.shape[0], image.shape[1], image.shape[2]))
  image = preprocess_input(image)
  print(image.shape)
  
  yhat = model.predict(image) # If the way is free then yhat ~ 1, if it is blocked yhat ~ 0
  print('Prediction is done!')
  
  # If the road is free then go forward, if it is blocked turn left
  if yhat > 0.5:
    subprocess.run('rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "forward"')
  else:
    subprocess.run('rostopic pub /jetbot_motors/cmd_str std_msgs/String --once "left"')
    
  time.sleep(0.001)

# Load in the model
model = models.load_model('jetson_inceptionv3.h5')
  
# Capturing the video stream with OpenCV
cap = cv2.VideoCapture('udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

while(True):
  avoid()
