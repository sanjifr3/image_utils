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

# Load the video
cap = cv2.VideoCapture(args['video'])

images = []
while cap.isOpened():
  ret,image = cap.read()

  if image is None: 
    break

  images.append(image)

done = False
i = 0

print "len(images): ", len(images)

while not done:
  cv2.imshow('image',images[i])
  pause = True
  while pause:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('d'):
      print "next"
      if i < len(images) - 1:
        i += 1
      pause = False
    elif key == ord('a'):
      print "previous"
      if i > 0:
        i -= 1
      pause = False;
    elif key == ord('q'):
      print "quit"
      pause = False
      done = True
# close all open windows
cv2.destroyAllWindows()