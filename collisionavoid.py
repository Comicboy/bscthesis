# Importing packages for the neural network
import numpy as np
import tensorflow
from tensorflow.keras import models
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from keras.applications.inception_v3 import preprocess_input

# Importing the packages for the jetson
from jetbot.jetbot import Robot
from jetcam.csi_camera import CSICamera
import time

# Configure the runtime
config = tensorflow.compat.v1.ConfigProto()
config.gpu_options.allow_growth = True
config.gpu_options.per_process_gpu_memory_fraction = 0.4
session = tensorflow.Session(config=config)

# Load in the model
model = models.load_model('jetson_inceptionv3.h5')

# Create camera with compatible output dimensions (299,299)
camera = CSICamera(width=299, height=299, capture_width=1080, capture_height=720, capture_fps=30)

# initializing the motors
robot = Robot()

def avoid():
  image = camera.read() # reads in the current frame into a (299,299,3) numpy array
  image = image.reshape((1, image.shape[0], image.shape[1], image.shape[2]))
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
