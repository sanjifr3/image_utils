#!/usr/bin/python2.7
import argparse
import numpy as np
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", required=True, help="Path to video")
args = vars(ap.parse_args())

cap = cv2.VideoCapture(args['video'])

iDir = '.'.join(args['video'].split('.')[:-1]) + '/'


print 'Images will be saved to ', iDir

ctr = 1
while cap.isOpened():
  ret, image = cap.read()

  if ret:
    cv2.imwrite(iDir + str(ctr) + '.jpg')
    ctr+=1

  else:
    break

cap.release()