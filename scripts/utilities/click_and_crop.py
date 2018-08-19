#!/usr/bin/python2.7
#import the necessary packages
import argparse
import cv2

refPt = []
cropping = False

def click_and_crop(event, x, y, flags, param):
  # Grab references to the global variables
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

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i","--image", required=True, help="Path to the image")
args = vars(ap.parse_args())

# Load the image
image = cv2.imread(args["image"])
clone = image.copy()

cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)

# Keep looping until the 'q' key is pressed
while True:
  #dislay the image and wait for a keypress
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
  roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
  cv2.imshow("ROI",roi)
  cv2.waitKey(0)

# close all open windows
cv2.destroyAllWindows()
