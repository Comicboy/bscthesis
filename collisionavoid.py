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
model = models.load_model('jetson_inceptionv3.pb')

# Create camera with compatible output dimensions (299,299)
camera = CSICamera(width=299, height=299, capture_width=1080, capture_height=720, capture_fps=30)

def avoid():
  image = camera.read() # reads in the current frame into a (299,299,3) numpy array
  image = preprocess_input(image)
  
  yhat = model.predict(image)
  label = decode_predictions(yhat) # Decoding the made prediction and selecting the highest probability
  label = label[0][0]
  
  prediction = label[2] # The probability of 
