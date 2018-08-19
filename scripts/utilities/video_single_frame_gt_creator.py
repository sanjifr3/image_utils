#!/usr/bin/python2.7
import argparse
import rospkg
import numpy as np
import cv2
from os import listdir

# Default Parameters
frameCheckInterval = 1;
multiUsers = False;

videosDir = '/videos/'
gtDir = '/gt/'
videos_to_convert = ['Combined_NULL_N_001_depth','Combined_NULL_N_000_depth']

rospack = rospkg.RosPack()
packagePath = rospack.get_path('social_robot')

videosDir = packagePath + '/validation/' + videosDir
gtDir = packagePath + '/validation/' + gtDir

# Global Variables
refPt = []
cropping = False

def click_and_crop(event, x, y, flags, param):
  global refPt, cropping

  # If the left mouse button is clicked, record the starting
  # (x,y) coordinates and indicate that cropping is being
  # performed

  if event == cv2.EVENT_LBUTTONDOWN:
    refPt = [(x,y)]
    cropping = True

  # check to see if the left mouse button was released
  elif event == cv2.EVENT_LBUTTONUP:
    # record the ending (x,y) coordinates and indicate that 
    # cropping operation is finished
    refPt.append((x,y))
    cropping = False

    # draw a rectangle around the region of interest
    cv2.rectangle(image, refPt[0], refPt[1], (0,255,0), 2)
    cv2.imshow("image", image)

# Sort points to the given format
def sortPoints(pts):
  [left,top] = np.amin(pts, axis=0)
  [right,bottom] = np.amax(pts, axis=0)

  return [(left,top),(right,bottom)]

# Get euclidean Distance
def euclideanDistance(pts):
  x_sq = (pts[0][0] - pts[1][0])**2
  y_sq = (pts[0][1] - pts[1][1])**2

  return np.sqrt(x_sq + y_sq)

# Print results to file
def printToFile(f,numOfUsers,framenum, names, pts):
  f.write(str(framenum))
  if numOfUsers == 0:
    f.write(',Environment,0,0,0,0\n')
    return
  for i in range(numOfUsers):
    f.write(','+names[i]+','+str(pts[i][0][0])+','+str(pts[i][0][1])+','+str(pts[i][1][0])+','+str(pts[i][1][1]))
  f.write('\n')

ctr = 0
targetName = "person"
for file in videos_to_convert:
  ctr += 1
  print '\nprogress: ', round(float(ctr) / float(len(videos_to_convert)),2)
  print ' ' + file

  if (file == "Combined_NULL_N_001_depth"):
    numTargets = 1
  else:
    numTargets = 4

  f = open(gtDir + file + '.csv', 'w')

  f.write('VideoName,' + file + '\n')
  
  f.write('i')
  for i in range(numTargets):
    f.write(',name,left,top,right,bottom')
  f.write('\n')

  cap = cv2.VideoCapture(videosDir + file + '.avi')
  cap2 = cv2.VideoCapture(videosDir + file[:-6] + '.avi')

  cv2.namedWindow('image')
  cv2.setMouseCallback('image', click_and_crop)
  framenum = 0

  previousFrameRef = [0]*numTargets
  currentFrameRef = []
  previousNames = []
  currentNames = []

  first_run = True
  frameCheckIntervalIndx = 0

  while cap.isOpened():
    ret, image = cap.read()
    ret2, image2 = cap2.read()

    if image is None:
      break
    if image2 is None:
      break

    cv2.imshow('image',image)
    cv2.imshow('image2',image2)
    cv2.waitKey(0)
    continue

    clone = image.copy()

    refPt = []

    if not first_run:
      currentFrameRef = previousFrameRef

    if frameCheckIntervalIndx == frameCheckInterval:
      if currentFrameRef == previousFrameRef:
        while True:
          for i in range(len(currentFrameRef)):
            cv2.rectangle(image, currentFrameRef[i][0], currentFrameRef[i][1], (255,0,0), 2)
            cv2.putText(image, currentNames[i], (currentFrameRef[i][0][0],currentFrameRef[i][0][1]-10), 1, 1.2, (255,0,0))
          cv2.imshow("image", image)
          key = cv2.waitKey(1) & 0xFF

          if key == ord("c"):
            break
          
          elif key == ord("r"):
            currentFrameRef = []
            currentNames = []
            image = clone.copy()
            break

          elif key == ord("s"):
            currentFrameRef = []
            image = clone.copy()
            break

      frameCheckIntervalIndx = 0

    while len(currentFrameRef) < numTargets:
      #image = clone.copy()
      while True:
        # Keep looping until the 'q' key is pressed
        cv2.imshow("image", image)
        key = cv2.waitKey(1) & 0xFF

        # If the 'r' key is pressed, reset the cropping region
        if key == ord("r"):
          image = clone.copy()

        # If the 'c' key is pressed, break from the loop
        elif key == ord('c'):
          break

        elif key == ord('s'):
          refPt = [(0,0),(0,0)]
          break

      # If there are two reference points, then crop the region of interest
      # from the image, and display it
      if len(refPt) == 2:
        roi = []
        refPt = sortPoints(refPt)
        
        if euclideanDistance(refPt) < 10 and euclideanDistance(refPt) > 0: 
          continue
        roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
        
        currentFrameRef.append(refPt)
        if len(currentNames) != numTargets:
          name = targetName + "_" + str(len(currentNames))
          currentNames.append(name)

        cv2.putText(roi, name, (0,15), 1, 1.0, (0,255,0))
        if ( euclideanDistance(refPt) != 0):
          cv2.imshow("ROI_" + str(len(currentFrameRef)),roi)
        cv2.waitKey(1)
        
    print framenum, currentFrameRef, currentNames
    printToFile (f, numTargets, framenum, currentNames, currentFrameRef)
    framenum+=1
    previousFrameRef = currentFrameRef
    previousNames = currentNames
    first_run = False
    frameCheckIntervalIndx += 1

  cv2.destroyAllWindows()
  cv2.waitKey(1)
  f.close()
  cap.release()