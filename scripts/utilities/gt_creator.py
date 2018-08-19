#!/usr/bin/python2.7
import argparse
import numpy as np
import cv2

# Default Parameters
numOfUsers = 1
frameCheckInterval = 50;

# Argument Parser
ap = argparse.ArgumentParser()
#ap.add_argument("-i","--image", required=True, help="Path to the image")
ap.add_argument("-v","--video", required=True, help="Path to video")
ap.add_argument("-n", "--num_users", required=False, help="Number of users")
ap.add_argument("-s", "--frame-check-interval", required=False, help="Interval of frames to check for updates")
args = vars(ap.parse_args())

videoName = args['video'].split('/')[-1]
fileName = args['video'].split('.')[0]

# Update params from command line
if args['num_users'] is not None:
  numOfUsers = int(args['num_users'])

if args['frame_check_interval'] is not None:
  frameCheckInterval = int(args['frame_check_interval'])

# Initialize file
f = open(fileName + '.csv','w')

f.write('VideoName,' + videoName + '\n')
f.write('NumberofUsers,' + str(numOfUsers) + '\n\n')

f.write('i')
for i in range(numOfUsers):
  f.write(',name,left,top,right,bottom')
f.write('\n')

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
def printToFile(framenum, names, pts):
  f.write(str(framenum))
  for i in range(numOfUsers):
    f.write(','+names[i]+','+str(pts[i][0][0])+','+str(pts[i][0][1])+','+str(pts[i][1][0])+','+str(pts[i][1][1]))
  f.write('\n')

# Load the video
cap = cv2.VideoCapture(args['video'])

cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
framenum = 0

previousFrameRef = [0]*numOfUsers
currentFrameRef = []
previousNames = []
currentNames = []

first_run = True
frameCheckIntervalIndx = 0

while cap.isOpened():
  
  ret, image = cap.read()
  if image is None:
    break
  clone = image.copy()
  
  refPt = []
  name = ''

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

  while len(currentFrameRef) < numOfUsers:
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

    # If there are two reference points, then crop the region of interest
    # from the image, and display it
    if len(refPt) == 2:
      roi = []
      refPt = sortPoints(refPt)
      
      if euclideanDistance(refPt) < 10: 
        continue
      roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
      
      currentFrameRef.append(refPt)
      if len(currentNames) != numOfUsers:
        name = raw_input('Name: ')
        currentNames.append(name)

      cv2.putText(roi, name, (0,15), 1, 1.0, (0,255,0))
      cv2.imshow("ROI_" + str(len(currentFrameRef)),roi)
      cv2.waitKey(1)
      
  print framenum, currentFrameRef, currentNames
  printToFile (framenum, currentNames, currentFrameRef)
  framenum+=1
  previousFrameRef = currentFrameRef
  previousNames = currentNames
  first_run = False
  frameCheckIntervalIndx += 1

# close all open windows
cv2.destroyAllWindows()
f.close()