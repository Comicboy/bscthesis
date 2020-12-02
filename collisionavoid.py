# Importing packages for the neural network
import numpy as np
from tensorflow.keras import models
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from keras.applications.inception_v3 import preprocess_input, decode_predictions

# Importing the packages for the jetson
from jetbot.jetbot import Robot
from jetcam.csi_camera import CSICamera
import time

# Load in the model
model = models.load_model('jetson_inceptionv3.h5')

# Create camera with compatible output dimensions (299,299)
camera = CSICamera(width=299, height=299, capture_width=1080, capture_height=720, capture_fps=30)

# initializing the motors
robot = Robot()

def avoid():
  image = camera.read() # reads in the current frame into a (299,299,3) numpy array
  image = preprocess_input(image)
  
  yhat = model.predict(image) # If the way is free then yhat ~ 1, if it is blocked yhat ~ 0
  
  # If the road is free then go forward, it it is blocked turn left
  if yhat > 0.5:
    robot.forward(0.7)
  else:
    robot.left(0.7)
    
  time.sleep(0.001)

# Calling the avoid function in an infinite loop
while(True):
  avoid()
