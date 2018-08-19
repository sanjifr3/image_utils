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

writer = cv2.VideoWriter(args['video'].split('Blueberry.')[0] + 'Results.avi', 
	cv2.cv.CV_FOURCC(*'DIVX'), 30, (640, 480), True)

images = []
while cap.isOpened():
  ret,image = cap.read()

  if image is None: 
    break

  images.append(image)

done = False
i = 0

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
      pause = False

    elif key == ord('q'):
      print "quit"
      pause = False
      done = True

    elif key == ord('z'):
    	print "save 2 frames"
    	for j in range(2):
				writer.write(images[i])
    	if i < len(images) -1:
  			i += 1
			pause = False

    elif key == ord('x'):
    	print "save 15 frames"
    	for j in range(15):
				writer.write(images[i])
    	if i < len(images) -1:
  			i += 1
			pause = False

    elif key == ord('c'):
    	print "save 30 frames"
    	for j in range(30):
				writer.write(images[i])
    	if i < len(images) -1:
  			i += 1
			pause = False
