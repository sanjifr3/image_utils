#!/usr/bin/python2.7
import argparse
import rospkg
import numpy as np
import cv2
import glob
from os import listdir

# Default Parameters
templDir = '/templates/'
skelDir = '/skeleton_templates/'

rospack = rospkg.RosPack()
packagePath = rospack.get_path('social_robot')

# Argument Parser
ap = argparse.ArgumentParser()
#ap.add_argument("-i","--image", required=True, help="Path to the image")
ap.add_argument("-d","--dir", required=True, help="Name of parent directory")
args = vars(ap.parse_args())

# Update params from command line
if args['dir'] is None:
  print "Directory not specified"
  exit(1)

templDir = packagePath + '/results/' + args['dir'] + templDir
skelDir = packagePath + '/results/' + args['dir'] + skelDir

templList = listdir(templDir)
templList = [file[:-4] for file in templList]

skelList = listdir(skelDir)
skelList = [file[:-9] for file in skelList]

fileList = [file for file in templList if file not in skelList]

for templPath in glob.glob(templDir + "/*.png"):
  img = cv2.imread(templPath,0)
  size = np.size(img)
  skel = np.zeros(img.shape,np.uint8)
   
  ret,img = cv2.threshold(img,127,255,0)
  element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
  done = False
   
  while( not done):
      eroded = cv2.erode(img,element)
      temp = cv2.dilate(eroded,element)
      temp = cv2.subtract(img,temp)
      skel = cv2.bitwise_or(skel,temp)
      img = eroded.copy()
   
      zeros = size - cv2.countNonZero(img)
      if zeros==size:
        done = True
  cv2.imwrite(skelDir + templPath[len(templDir):-4] + '_skel.png',skel)