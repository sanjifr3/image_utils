#!/usr/bin/python2.7
import argparse
import numpy as np
import cv2

# Argument Parser
ap = argparse.ArgumentParser()
#ap.add_argument("-i","--image", required=True, help="Path to the image")
ap.add_argument("-v","--video", required=True, help="Path to video")
args = vars(ap.parse_args())

# Global Variables
refPt = []
cropping = False

fps = 24

time_per_frame = 1*fps

# Load the video
cap = cv2.VideoCapture(args['video'])

writer = cv2.VideoWriter(args['video'].split('.')[0] + '_slow.avi', 
	cv2.cv.CV_FOURCC(*'DIVX'), fps, (640, 480), True)

images = []
while cap.isOpened():
  ret,image = cap.read()

  if image is None: 
    break

  for i in range(time_per_frame):
    writer.write(image)