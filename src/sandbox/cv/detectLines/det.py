# image pre-processing: mask (determine relevant areas of image), threshold (determine which colors are lanes)
# lane detection: canny edge detection -> Hough line 

import cv2
#import matplotlib.pyplot as plt
import numpy as np
from numpy.core.fromnumeric import put
windowName = 'image'

resolutionRho = 1
resolutionTheta = np.pi/180
threshold = 20
minLineLength = 20
maxLineGap = 20
gaussianBlurSize = 5
cannyFirstThreshold = 70
cannySecondThreshold = 150
alpha = 1.5; beta = 1 

resolutionRhoRange = (1, 4)
resolutionThetaRange = (np.pi/180, 10)
thresholdRange = (0, 200)
minLineLengthRange = (0, 100)
maxLineGapRange = (0, 200)
gaussianBlurRange = (1, 101) 
gaussianBlurSliderRange = (0, 50)
cannyFirstThresholdRange = (0, 200)
cannySecondThresholdRange = (0, 200)
contrastRange = (0, 10)
contrastSliderRange = (0, 1)


curImg = -1
img = None
imgOriginal = None

# define masked region 
def region(img): 
  height, width = img.shape
  trap = np.array([[(0, 525), (1250, 0), (1920, 0), (width, 775)]])
  mask = np.zeros_like(img)
  mask = cv2.fillPoly(mask, trap, 255)
  mask = cv2.bitwise_and(img, mask)
  return mask

# Gaussian blur parameter is ksize, tuple of odd integers only. Need to make trackbar for ksize adjustment. 
def getBlurredImg(inputImg): 
  global gaussianBlurSize
  cpy = inputImg.copy()
  img = cv2.GaussianBlur(cpy, (gaussianBlurSize, gaussianBlurSize), 0)
  return img 

# Canny edges are taken after Gaussian blurring. Returns the Canny edge image for Hough lines to be taken from. 
def getEdges(inputImg):
  global cannyFirstThreshold, cannySecondThreshold
  cpy = inputImg.copy()
  edges = cv2.Canny(cpy, cannyFirstThreshold, cannySecondThreshold)
  return edges 

def getHoughLines(inputImg):
  global resolutionRho, resolutionTheta, threshold, minLineLength, maxLineGap
  cpy = inputImg.copy()
  lines = cv2.HoughLinesP(cpy, resolutionRho, resolutionTheta, threshold, minLineLength = minLineLength, maxLineGap = maxLineGap)
  return lines
  
def applyLinesToImage(img, lines):
  cpy = img.copy()
  for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(cpy, (x1, y1), (x2, y2), (255, 0, 0), 3)

  #apply line count on to text
  cv2.putText(cpy, f"lines:{len(lines)}", (5,240), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0,255), 3) 
  return cpy

def applyValuesToImage(img):
  global threshold, minLineLength, maxLineGap
  cpy = img.copy()
  cv2.putText(cpy, f"threshold:{threshold}", (5,40), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3) 
  cv2.putText(cpy, f"minLineLength:{minLineLength}", (5,80), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3) 
  cv2.putText(cpy, f"maxLineGap:{maxLineGap}", (5,120), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3) 
  cv2.putText(cpy, f"kSizeBlur:{gaussianBlurSize}", (5,160), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
  cv2.putText(cpy, f"Canny Thresholds:{cannyFirstThreshold} to {cannySecondThreshold}", (5, 200), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
  cv2.putText(cpy, f"Contrast:{alpha}", (5, 240), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
  return cpy

def updateResolutionRho(val):
  global resolutionRho
  resolutionRho= val
  updateImage()

def updateResolutionTheta(val):
  global resolutionTheta
  resolutionTheta = val
  updateImage()

def updateThreshold(val):
  global threshold 
  threshold = val
  updateImage()

def updateMinLineLength(val):
  global updateMinLineLength
  updateMinLineLength = val
  updateImage()

def updateMaxLineGap(val):
  global maxLineGap
  maxLineGap = val
  updateImage()

def updatekSizeBlur(val): 
  global gaussianBlurSize
  gaussianBlurSize = val * 2 + 1
  updateImage()

def updateContrastThreshold(val): 
  global alpha
  alpha = val * 0.1 + 1
  updateImage()

def updateCannyEdgeThreshold1(val):
  global cannyFirstThreshold
  cannyFirstThreshold = val 
  updateImage()

def updateCannyEdgeThreshold2(val):
  global cannySecondThreshold
  cannySecondThreshold = val 
  updateImage()

def updateCannyEdgeThreshold1(val):
  global cannyFirstThreshold
  cannyFirstThreshold = val 
  updateImage()

def updateCannyEdgeThreshold2(val):
  global cannySecondThreshold
  cannySecondThreshold = val 
  updateImage()


def updateImage():
  print("updating image")
  global img
  blurred = getBlurredImg(img)
  edges = getEdges(blurred)
  space = region(edges)
  lines = getHoughLines(space)
  out = applyLinesToImage(imgOriginal, lines)
  out = applyValuesToImage(out)
  cv2.imshow(windowName, out)

def getNextImage():
  global img, imgOriginal, curImg
  curImg += 1
  img = cv2.imread(f"./sandbox/cv/detectLines/small/aldrich{curImg}.png",cv2.IMREAD_UNCHANGED)
  if img is None:
    curImg = 0
    img = cv2.imread(f"./sandbox/cv/detectLines/small/aldrich{curImg}.png",cv2.IMREAD_UNCHANGED)
  imgOriginal = img.copy()

  # Do preprocessing here

  # contrast/low-light adjust
  mean = cv2.mean(img)
  imgMean = (mean[0] + mean[1] + mean[2]) / 3 
  if (imgMean < 127): 
    img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)   
 
  # convert image to gray scale
  img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

  # apply image thresholding
  # ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

#Init and display the first image
cv2.namedWindow(windowName)
#cv2.createTrackbar('ResRho', windowName, resolutionRhoRange[0], resolutionRhoRange[1], updateResolutionRho)
#cv2.createTrackbar('ResTheta', windowName, resolutionThetaRange[0], resolutionThetaRange[1], updateResolutionTheta)

# for Hough Lines
cv2.createTrackbar('HoughThreshold', windowName, thresholdRange[0], thresholdRange[1],  updateThreshold)
cv2.createTrackbar('HoughMinLineLength', windowName, minLineLengthRange[0], minLineLengthRange[1], updateMinLineLength)
cv2.createTrackbar('HoughMaxLineGap', windowName, maxLineGapRange[0], maxLineGapRange[1], updateMaxLineGap)

# Gaussian - only works with odd values
cv2.createTrackbar('GaussianBlurKSize', windowName, gaussianBlurSliderRange[0], gaussianBlurSliderRange[1], updatekSizeBlur)

# Canny
cv2.createTrackbar('CannyEdgeThreshold1', windowName, cannyFirstThresholdRange[0], cannyFirstThresholdRange[1], updateCannyEdgeThreshold1)
cv2.createTrackbar('CannyEdgeThreshold2', windowName, cannySecondThresholdRange[0], cannySecondThresholdRange[1], updateCannyEdgeThreshold2)

# alpha brightness adjustment 
cv2.createTrackbar('ContrastAdjustment', windowName, contrastRange[0], contrastRange[1], updateContrastThreshold)

getNextImage()
updateImage()

while True:
  if(cv2.waitKey(0) == ord('d')):
    getNextImage()
    updateImage()
  else:
    cv2.destroyAllWindows()
    exit()
