# image pre-processing: mask (determine relevant areas of image), threshold (determine which colors are lanes)
# lane detection: canny edge detection -> Hough line 

from audioop import avg
import cv2
#import matplotlib.pyplot as plt
import numpy as np
import time
from numpy.core.fromnumeric import put
import laneDetect
windowName = 'image'
IMAGE_WIDTH = 480
IMAGE_HEIGHT = 640

alpha = 1.5
beta = 1
contrastAdjustThreshold = 127

# DEFAULT 
# resolutionRho = 1
# # resolutionTheta = np.pi/180
# resolutionTheta = 1
# threshold = 20
# minLineLength = 20
# maxLineGap = 20
# gaussianBlurSize = 5
# cannyFirstThreshold = 55
# cannySecondThreshold = 95
# rectTopMaskHeight = 0
# rectBottomMaskHeight = 0
# ksize = 1

# alpha=1.0
# beta=1.1
# contrastAdjustThreshold=127
# resolutionRho=3
# resolutionTheta=1
# threshold=4
# minLineLength=0
# maxLineGap=0
# ksize=3
# cannyFirstThreshold=186
# cannySecondThreshold=70
# rectTopMaskHeight=256
# rectBottomMaskHeight=451
# resolutionRho2=3
# resolutionTheta2=1
# threshold2=4
# minLineLength2=0
# maxLineGap2=0

alpha=0.6
beta=0.4
contrastAdjustThreshold=127
resolutionRho=9
resolutionTheta=0.6632251157578452
threshold=75
minLineLength=7
maxLineGap=30
ksize=3
cannyFirstThreshold=67
cannySecondThreshold=88
rectTopMaskHeight=153
rectBottomMaskHeight=640
resolutionRho2=3
resolutionTheta2=2.356194490192345
threshold2=200
minLineLength2=0
maxLineGap2=41

doGscale = 1
doContrastAdjustment = 1
doGaussianBlur = 1
doCanny = 1
doHough = 1
doHough2 = 1
doRegion = 1


doGscaleRange = (0, 1)
doContrastAdjustmentRange = (0, 1)
doGaussianBlurRange = (0, 1)
doCannyRange = (0, 1)
doHoughRange = (0, 1)
doRegionRange = (0, 1)
alphaRange = (10, 40)
betaRange = (10, 40)
resolutionRhoRange = (1, 10)
resolutionThetaRange = (1, 360)
thresholdRange = (0, 200)
minLineLengthRange = (0, 100)
maxLineGapRange = (0, 200)
gaussianBlurRange = (1, 101) 
gaussianBlurSliderRange = (0, 50)
cannyFirstThresholdRange = (0, 400)
cannySecondThresholdRange = (0, 400)
contrastRange = (0, 10)
contrastSliderRange = (0, 1)
rectTopMaskHeightRange = (0, IMAGE_HEIGHT)
rectBottomMaskHeightRange = (0, IMAGE_HEIGHT)
ksizeRange = (1, 21)

vcap=None
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

# def getHoughLines(inputImg):
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

def updateAlpha(val):
    global alpha
    val = val/10.0
    print(val)
    alpha = val
    updateImage()

def updateBeta(val):
    global beta
    val = val/10.0
    print(val)
    beta = val
    updateImage()

def updateResolutionRho(val):
    print(val)
    global resolutionRho
    resolutionRho = val
    updateImage()

def updateResolutionTheta(val):
    global resolutionTheta
    resolutionTheta = val * np.pi/180
    print(str(resolutionTheta))
    updateImage()

def updateThreshold(val):
    print(val)
    global threshold 
    threshold = val
    updateImage()

def updateMinLineLength(val):
    print(val)
    global minLineLength
    minLineLength = val
    updateImage()

def updateMaxLineGap(val):
    print(val)
    global maxLineGap
    maxLineGap = val
    updateImage()

def updatekSizeBlur(val): 
    global ksize
    ksize = val * 2 + 1
    print(str(ksize))
    updateImage()

def updateContrastThreshold(val): 
    print(val)
    global alpha
    alpha = val * 0.1 + 1
    updateImage()

def updateCannyEdgeThreshold1(val):
    print(val)
    global cannyFirstThreshold
    cannyFirstThreshold = val 
    updateImage()

def updateCannyEdgeThreshold2(val):
    print(val)
    global cannySecondThreshold
    cannySecondThreshold = val 
    updateImage()

def updateCannyEdgeThreshold1(val):
    print(val)
    global cannyFirstThreshold
    cannyFirstThreshold = val 
    updateImage()

def updateCannyEdgeThreshold2(val):
    print(val)
    global cannySecondThreshold
    cannySecondThreshold = val 
    updateImage()



def updateDoGscale(val):
    print(val)
    global doGscale
    doGscale = val
    updateImage()

def updateDoContrastAdjustment(val):
    print(val)
    global doContrastAdjustment
    doContrastAdjustment = val
    updateImage()

def updateDoGaussianBlur(val):
    print(val)
    global doGaussianBlur
    doGaussianBlur = val
    updateImage()

def updateDoCanny(val):
    print(val)
    global doCanny
    print(val)
    doCanny = val
    updateImage()

def updateDoHough(val):
    print(val)
    global doHough
    doHough = val
    updateImage()

def updateDoRegion(val):
    print(val)
    global doRegion
    doRegion = val
    updateImage()

def updateRectTopMaskHeight(val):
    print(val)
    global rectTopMaskHeight
    rectTopMaskHeight = val
    updateImage()

def updateRectBottomMaskHeight(val):
    print(val)
    global rectBottomMaskHeight
    rectBottomMaskHeight = val
    updateImage()

def updateImageOriginal():
    global img
    cv2.imshow(windowName, img)

def updateResolutionRho2(val):
    global resolutionRho2
    resolutionRho2= val
    updateImage()
def updateResolutionTheta2(val):
    global resolutionTheta2
    resolutionTheta2 = val * np.pi/180
    print(resolutionTheta2)
    updateImage()
def updateThreshold2(val):
    global threshold2
    threshold2= val
    updateImage()
def updateMinLineLength2(val):
    global minLineLength2
    minLineLenth2 = val
    updateImage()
def updateMaxLineGap2(val):
    global maxLineGap2
    maxLineGap2= val
    updateImage()
def updateDoHough2(val): 
    global doHough2
    doHough2 = val
    updateImage()

def updateImage():
    global img
    cpyImg = img.copy()
#   blurred = getBlurredImg(img)
#   edges = getEdges(blurred)
#   space = region(edges)
#   lines = getHoughLines(space)
#   out = applyLinesToImage(imgOriginal, lines)
#   out = applyValuesToImage(out)
#   cv2.imshow(windowName, out)
    # cpy = img.copy()
    # cpyColor = img.copy()


    # if doContrastAdjustment:
    #     cpy = laneDetect.applyContrastAdjustment(cpy, alpha, beta, contrastAdjustThreshold)
    
    # if doGscale:
    #     cpy = laneDetect.applyGscale(cpy)

    # if doGaussianBlur:
    #     cpy = laneDetect.applyGaussianBlur(cpy, ksize)

    # if doCanny:
    #     edges = laneDetect.getCannyEdges(cpy, cannyFirstThreshold, cannySecondThreshold)
    #     cpy = edges

    # if doRegion:
    #     #cpy = laneDetect.region(cpy)
    #     cpy = laneDetect.rectTopDownMask(cpy, rectTopMaskHeight)
    #     cpy = laneDetect.rectBottomUpMask(cpy, rectBottomMaskHeight)
    #     #TODO


    # if doHough:
    #     edges = laneDetect.getHoughLinesP(cpy, resolutionRho, resolutionTheta, threshold, mll=minLineLength, mlg=maxLineGap)
    #     cpyColor = laneDetect.drawHoughLines(cpyColor, edges)

    # # average lines after doHough1
    # avg_lines = laneDetect.average(cpyColor, edges)

    # cpyColor = laneDetect.drawAverageLines(cpyColor, avg_lines)

    # angle = laneDetect.findSteeringAngle(cpyColor, avg_lines=avg_lines, drawMidpointLines=True)
    # print(angle)

    # # calculate steering angle
    # #steering_angle = laneDetect.findSteeringAngle(cpyColor, avg_lines)
    # #cv2.putText(cpyColor, "Steering angle: " + str(steering_angle), (10, 25), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)


    print(laneDetect.modelTest1(cpyImg))
    #cv2.imshow(windowName, cpy)
    cv2.imshow(windowName, cpyImg)

    
    
def initCapDevice():
    global vcap
    vcap = cv2.VideoCapture("rtsp://192.168.0.102:8554/test")
    getNextImage()

def getImageFromFile():
    global img, imgOriginal, curImg
    img = cv2.imread(f"./sandbox/cv/detectLines/small/aldrich{curImg}.png",cv2.IMREAD_UNCHANGED)

def getNextImage():
    #https://stackoverflow.com/questions/57716962/difference-between-video-capture-read-and-grab
    #https://answers.opencv.org/question/197177/skip-frames-and-seek-to-end-of-rtsp-stream/
    global img, imgOriginal, curImg, vcap

    ret, frame = vcap.read()
    if ret:
        img = frame
        cv2.imshow(windowName, frame)

    else:
        print("Error Getting Image from rtsp")

def retrieveAndShowImage():
    #https://stackoverflow.com/questions/57716962/difference-between-video-capture-read-and-grab
    #https://answers.opencv.org/question/197177/skip-frames-and-seek-to-end-of-rtsp-stream/
    global img, imgOriginal, curImg, vcap

    ret, frame = vcap.retrieve()
    if ret:
        img = frame
        cv2.imshow(windowName, frame)

    else:
        print("Error Getting Image from rtsp")


def printSettings():
    print("alpha="+str(alpha))
    print("beta="+str(beta))
    print("contrastAdjustThreshold="+str(contrastAdjustThreshold))
    print("resolutionRho="+str(resolutionRho))
    print("resolutionTheta="+str(resolutionTheta))
    print("threshold="+str(threshold))
    print("minLineLength="+str(minLineLength))
    print("maxLineGap="+str(maxLineGap))
    print("ksize="+str(ksize))
    print("cannyFirstThreshold="+str(cannyFirstThreshold))
    print("cannySecondThreshold="+str(cannySecondThreshold))
    print("rectTopMaskHeight="+str(rectTopMaskHeight))
    print("rectBottomMaskHeight="+str(rectBottomMaskHeight))
    print("resolutionRho2="+str(resolutionRho2))
    print("resolutionTheta2="+str(resolutionTheta2))
    print("threshold2="+str(threshold2))
    print("minLineLength2="+str(minLineLength2))
    print("maxLineGap2="+str(maxLineGap2))


#Init and display the first image
cv2.namedWindow(windowName)


# doContrastAdjustment = 1
# doGscale = 1
# doGaussianBlur = 1
# doCanny = 1
# doHough = 1
# doRegion = 1

cv2.createTrackbar('dGscale', windowName, doGscaleRange[0], doGscaleRange[1], updateDoGscale)

cv2.createTrackbar('dContrastAdj', windowName, doContrastAdjustmentRange[0], doContrastAdjustmentRange[1], updateDoContrastAdjustment)
cv2.createTrackbar('alpha', windowName, alphaRange[0], alphaRange[1], updateAlpha)
cv2.createTrackbar('beta', windowName, betaRange[0], betaRange[1], updateBeta)



# Gaussian - only works with odd values
#cv2.createTrackbar('dGaussian', windowName, doGaussianBlurRange[0], doGaussianBlurRange[1], updateDoGaussianBlur)
#cv2.createTrackbar('GaussianBlurKSize', windowName, gaussianBlurSliderRange[0], gaussianBlurSliderRange[1], updatekSizeBlur)
cv2.createTrackbar('GaussianBlurKSize', windowName, ksizeRange[0], ksizeRange[1], updatekSizeBlur)

# Canny
cv2.createTrackbar('dCanny', windowName, doCannyRange[0], doCannyRange[1], updateDoCanny)
cv2.createTrackbar('Canny1EdgeThreshold1', windowName, cannyFirstThresholdRange[0], cannyFirstThresholdRange[1], updateCannyEdgeThreshold1)
cv2.createTrackbar('Canny2EdgeThreshold2', windowName, cannySecondThresholdRange[0], cannySecondThresholdRange[1], updateCannyEdgeThreshold2)

cv2.createTrackbar('dRegion', windowName, doRegionRange[0], doRegionRange[1], updateDoRegion)

cv2.createTrackbar('maskTop', windowName, rectTopMaskHeightRange[0], rectTopMaskHeightRange[1], updateRectTopMaskHeight)
cv2.createTrackbar('maskBotom', windowName, rectBottomMaskHeightRange[0], rectBottomMaskHeightRange[1], updateRectBottomMaskHeight)


# for Hough Lines
cv2.createTrackbar('dHough', windowName, doHoughRange[0], doHoughRange[1], updateDoHough)
#cv2.createTrackbar('ResRho', windowName, resolutionRhoRange[0], resolutionRhoRange[1], updateResolutionRho)
#cv2.createTrackbar('ResTheta', windowName, resolutionThetaRange[0], resolutionThetaRange[1], updateResolutionTheta)
cv2.createTrackbar('HoughThreshold', windowName, thresholdRange[0], thresholdRange[1],  updateThreshold)
cv2.createTrackbar('HoughMinLineLength', windowName, minLineLengthRange[0], minLineLengthRange[1], updateMinLineLength)
cv2.createTrackbar('HoughMaxLineGap', windowName, maxLineGapRange[0], maxLineGapRange[1], updateMaxLineGap)

cv2.createTrackbar('resolutionRho', windowName, resolutionRhoRange[0], resolutionRhoRange[1], updateResolutionRho)
cv2.createTrackbar('resolutionTheta', windowName, resolutionThetaRange[0], resolutionThetaRange[1], updateResolutionTheta)

# for Hough Lines
# cv2.createTrackbar('d2Hough', windowName, doHoughRange[0], doHoughRange[1], updateDoHough2)
# #cv2.createTrackbar('ResRho', windowName, resolutionRhoRange[0], resolutionRhoRange[1], updateResolutionRho)
# #cv2.createTrackbar('ResTheta', windowName, resolutionThetaRange[0], resolutionThetaRange[1], updateResolutionTheta)
# cv2.createTrackbar('Hough2Threshold', windowName, thresholdRange[0], thresholdRange[1],  updateThreshold2)
# cv2.createTrackbar('Hough2MinLineLength', windowName, minLineLengthRange[0], minLineLengthRange[1], updateMinLineLength2)
# cv2.createTrackbar('Hough2MaxLineGap', windowName, maxLineGapRange[0], maxLineGapRange[1], updateMaxLineGap2)

# cv2.createTrackbar('res2olutionRho', windowName, resolutionRhoRange[0], resolutionRhoRange[1], updateResolutionRho2)
# cv2.createTrackbar('res2olutionTheta', windowName, resolutionThetaRange[0], resolutionThetaRange[1], updateResolutionTheta2)










# alpha brightness adjustment 
#cv2.createTrackbar('ContrastAdjustment', windowName, contrastRange[0], contrastRange[1], updateContrastThreshold)

initCapDevice()
# while 1:
#     getNextImage()
#     time.sleep(1)
#updateImage()


def capSeekFrames(f):
    for i in range(0, f):
        vcap.grab();

def capSeekTillEnd():
    ret = vcap.grab()
    while ret != False:
        vcap.grab()


while True:
    k = cv2.waitKey(0)
    if(k == ord('d')):
        #getNextImage()
        capSeekFrames(10)
        retrieveAndShowImage()
        #updateImage()
    elif(k == ord('w')):
        updateImageOriginal()
    
    elif(k == ord('e')):
        updateImage()
    
    elif(k == ord('x')):
        printSettings()

    else:
        cv2.destroyAllWindows()
        exit()
