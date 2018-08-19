#!/usr/bin/python2.7
import argparse
import rospkg
import numpy as np
import cv2
from os import listdir

# Default Parameters
frameCheckInterval = 50
multiUsers = False
imgDir = 'images-merged/'
gtDir = 'gt/'

rospack = rospkg.RosPack()
packagePath = rospack.get_path('social_robot')

# Argument Parser
ap = argparse.ArgumentParser()
ap.add_argument("-m", "--multi-user", required=False, help="allow for multi-users")
args = vars(ap.parse_args())

imgDir = packagePath + '/validation/' + imgDir
gtDir = packagePath + '/validation/' + gtDir

# fileList = listdir(gtDir)

# for file in fileList:
#   print file
#   df = genfromtxt(gtDir + file, delimiter=',')
#   numPeople = df.shape[0]#len(df[:,0])
#   elements = df.size

#   f = open(gtDir + file,'w')

#   print df

#   if numPeople == elements:
#     numPeople = 1

#   print numPeople

#   if numPeople > 1:
#     for i in range(numPeople):
#       f.write(str(df[i,0])+','+str(df[i,1])+','+str(df[i,2])+','+str(df[i,3])+',person_'+str(i)+'\n')
#   else:
#     f.write((str(df[0])+','+str(df[1])+','+str(df[2])+','+str(df[3])+',person_0\n'))

# exit(1)

if args['multi_user'] is not None:
  multiUsers = True

imgList = listdir(imgDir)
imgList = [file.split('.')[0] for file in imgList]

gtList = listdir(gtDir)
gtList = [file.split('.')[0] for file in gtList]

fileList = [file for file in imgList if file not in gtList]

print fileList

# Global Variables
refPt = []
cropping = False

def filterDepth (depth_im):
  for i in range(depth_im.shape[0]):
    for j in range(depth_im.shape[1]):
      if depth_im[i][j]/1000.0 > 3.65:
        depth_im[i][j] = 0
      elif depth_im[i][j]/1000.0 < 1.5:
        depth_im[i][j] = 0

  return depth_im

from sklearn.preprocessing import normalize

def colorBySN (depth_im):

  sn_im = np.zeros((depth_im.shape[0],depth_im.shape[1],3))

  for i in range(depth_im.shape[0]):
    for j in range(depth_im.shape[1]):

      if i-1 < 0 or i+1 >= depth_im.shape[0]:
        sn_im[i][j] = [0,0,0]
        continue
      if j-1 < 0 or j+1 >= depth_im.shape[1]:
        sn_im[i][j] = [0,0,0]
        continue

      if depth_im[i][j] == 0:
        sn_im[i][j] = [0,0,0]
        continue

      # print i-1,i,i+1
      # print j-1,j,j+1
      dzdx = depth_im[i+1][j] - depth_im[i-1][j] / 2.0
      dzdy = depth_im[i][j+1] - depth_im[i][j-1] / 2.0
      total = np.sum(-dzdy + -dzdx + 1.0)

      if total == 0:
        sn_im[i][j] = [0,0,0]
        continue

      sn_im[i][j] = [-dzdx/total, -dzdy/total, 1.0/total]


  return sn_im

def eucDis(pt1,pt2):
  x_sq = (pt1[0] - pt2[0])**2
  y_sq = (pt1[1] - pt2[1])**2
  z_sq = (pt1[2] - pt2[2])**2

  return np.sqrt(x_sq + y_sq + z_sq)


def filterBySN (sn_im):
  rejected_vectors = []
  check_dist = 10
  tol = 0.1

  new_im = np.zeros(sn_im.shape)

  for i in range(sn_im.shape[0]):
    for j in range(sn_im.shape[1]):

      if i-check_dist < 0 or i+check_dist >= sn_im.shape[0]:
        new_im[i][j] = [0,0,0]
        continue
      elif j-check_dist < 0 or j+check_dist >= sn_im.shape[1]:
        new_im[i][j] = [0,0,0]
        continue
      elif eucDis(sn_im[i-check_dist][j],sn_im[i+check_dist][j]) < tol and eucDis(sn_im[i][j-check_dist],sn_im[i][j+check_dist]) < tol:
        rejected_vectors.append(sn_im[i][j])

      rejected = False
      for vect in rejected_vectors:
        if eucDis(vect,sn_im[i][j]):
          new_im[i][j] = [0,0,0]
          rejected=True

      if not rejected:
        new_im[i][j] = sn_im[i][j]

  return new_im

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
    cv2.rectangle(d_img, refPt[0], refPt[1], (0,255,0), 2)
    cv2.imshow("depth_im", d_img)

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
def printToFile(f, pts, names, numOfUsers, mc=False):
  if numOfUsers == 0:
    f.write('0,0,0,0,Environment\n')
    return
  if numOfUsers == 0 and mc:
    f.write('0,0,0,0,mcEnvironment\n')
    return
  for i in range(numOfUsers):
    f.write(str(pts[i][0][0]) + ',' + str(pts[i][0][1]) + ',' + str(pts[i][1][0]) + ',' + str(pts[i][1][1]) + ',' + names[i] + '\n')

## Main ## 
ctr = 0
for file in fileList:
  print '\nprogress: ', round(float(ctr) / float(len(fileList)),2)
  print ' ' + file
  ctr += 1
  numTargets = -1
  targetName = ''
  name = file.split('_')[0]
  
  if name == "Environment":
    numTargets = 0
    f = open(gtDir + file + '.csv', 'w')
    printToFile(f,0,0,0)
    f.close()
    continue
  if name == "mcEnvironment":
    numTargets = 0
    f = open(gtDir + file + '.csv','w')
    printToFile(f,0,0,0,True)
    f.close()
    continue
  if name == "Scene" or name == "Mixed":
    multiUsers = True
    name = "person"
  else:
    multiUsers = False

  img = cv2.imread(imgDir + file + '.jpg')
  d_img = cv2.imread(imgDir + file + '.tiff', cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)

  # sn_img = colorBySN(d_img)

  # cv2.imshow('original',d_img)
  # cv2.imshow('sn_img_x',sn_img[:,:,0])
  # cv2.imshow('sn_img_y',sn_img[:,:,1])
  # cv2.imshow('sn_img_z',sn_img[:,:,2])

  # sn_img = filterBySN(sn_img)

  # cv2.imshow('filtered_sn_img',sn_img)


  d_img = filterDepth(d_img)

  d_img = cv2.cvtColor(d_img, cv2.COLOR_GRAY2RGB)
  d_img = cv2.normalize(d_img, 0, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)


  cv2.namedWindow('rgb_im')
  cv2.namedWindow('depth_im')
  cv2.imshow('rgb_im',img)
  cv2.imshow('depth_im',d_img)
  cv2.waitKey(1)

  # if multiUsers:
  #   while numTargets == -1:
  #     try:
  #       cv2.imshow('rgb_im',img)
  #       cv2.imshow('depth_im',d_img)
  #       cv2.waitKey(0)
  #       numTargets = int(raw_input('How many people are in this file? '))
  #       print ' There are',numTargets,'people in this file'
  #     except Exception as e:
  #       print "Invalid number of targets!"
  # else:
  #   numTargets = 1

  if numTargets == 0:
    f = open(gtDir + file + '.csv', 'w')
    printToFile(f,0,0,0)
    f.close()

  f = open(gtDir + file + '.csv','w')

  cv2.namedWindow('depth_im')
  cv2.setMouseCallback('depth_im', click_and_crop)

  clone = d_img.copy()

  refPt = []

  allRefPts = []
  allNames = []

  done = False

  while not done:
    while True:
      cv2.imshow('depth_im',d_img)
      key = cv2.waitKey(1) & 0xFF

      if key == ord('r'):
        d_img = clone.copy()

      elif key == ord('c'):
        break

      elif key == ord('d'):
        done = True
        break

    if done:
      break

    if len(refPt) == 2:
      roi = []
      refPt = sortPoints(refPt)
      
      if euclideanDistance(refPt) < 10: 
        continue
      roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
      allRefPts.append(refPt)

      if not multiUsers:
        allNames.append(name)
      else:
        allNames.append(name + '_' + str(len(allNames)))

      cv2.putText(roi, name, (0,15), 1, 1.0, (0,255,0))
      cv2.imshow("ROI_" + str(len(allRefPts)),roi)
      cv2.waitKey(1)
        
  print allRefPts, allNames
  if len(allNames) > 0:
    printToFile (f, allRefPts, allNames, len(allNames))
  else:
    printToFile(f,0,0,0)
    
  cv2.waitKey(1)
  f.close()

cv2.destroyAllWindows()