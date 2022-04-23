# WORKING LANE DETECTION ALG #

import cv2
import matplotlib.pyplot as plt
import numpy as np

windowName = 'image'

# Default Values that Seem to Work #
# threshold = 40
# minLineLength = 75
# maxLineGap = 50 
# GaussianBlurKSize (3, 3) 
# canny thresholds = 55 to 95

alpha = 1.5; 
beta = 1; 

resolutionRho = 1
resolutionTheta = np.pi/180
threshold = 40
minLineLength = 75
maxLineGap = 50

resolutionRhoRange = (1, 4)
resolutionThetaRange = (np.pi/180, 10)
thresholdRange = (0, 200)
minLineLengthRange = (0, 100)
maxLineGapRange = (0, 200)
gaussBlurRange = (1, 101)

# function definitions 

# display: function for displaying image, used for testing processing steps
def display(img): 
    cv2.imshow(windowName, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows
    exit()

# region: define region of interest
def region(img): 
    height, width = img.shape
    trap = np.array([[(0, 525), (1250, 0), (1920, 0), (width, 775)]])
    mask = np.zeros_like(img)
    mask = cv2.fillPoly(mask, trap, 255)
    mask = cv2.bitwise_and(img, mask)
    return mask

def make_points(img, average): 
    slope, y_int = average 
    y1 = img.shape[0]
    y2 = int(y1 * (1/4))
    x1 = int((y1 - y_int) // slope )
    x2 = int((y2 - y_int) // slope )
    return np.array([x1, y1, x2, y2])

def average(img, lines): 
    slopes = []
    for line in lines: 
        x1, y1, x2, y2  = np.reshape(line, 4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_int = parameters[1]
        slopes.append((slope, y_int))

    # filter horizontal slopes
    HORIZONTAL_THRESHOLD = 0.00001
    slopes = [s for s in slopes if s[0] > HORIZONTAL_THRESHOLD or s[0] < -HORIZONTAL_THRESHOLD]

    left = [s for s in slopes if s[0] < 0]
    right = [s for s in slopes if s[0] > 0]

    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)
    print("left avg: " + str(left_avg))
    print("right avg: " + str(right_avg))

    left_line = make_points(img, left_avg)
    right_line = make_points(img, right_avg)

    print(left_line)
    print(right_line)
    return np.array([left_line, right_line])

def applyLinesToImage(img, lines):
    cpy = img.copy()
    for line in lines:
        x1, y1, x2, y2 = line
        cv2.line(cpy, (x1, y1), (x2, y2), (255, 0, 0), 15)
    return cpy

# import unadulterated image 
"""
img = cv2.imread("./src/sandbox/cv/detectLines/small/aldrich3.png", cv2.IMREAD_UNCHANGED)
img_original = img.copy(); 

mean = cv2.mean(img)
print("mean: {}".format(mean)) 
imgMean = (mean[0] + mean[1] + mean[2]) / 3; 

# contrast increase for low light
if (imgMean < 127): 
    contrast = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
    grey = cv2.cvtColor(contrast, cv2.COLOR_BGR2GRAY)
else: 
    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# gaussian blur 
# (11, 11) kernel dimensions blur it such that it eliminates other slow changing noise (contrast indicator)
gauss = cv2.GaussianBlur(grey, (3, 3), 0)

# canny edge detection 
edges = cv2.Canny(gauss, 55, 95)

# masked image 
space = region(edges)

# get Hough lines + average
lines = cv2.HoughLinesP(space, resolutionRho, resolutionTheta, threshold, minLineLength = minLineLength, maxLineGap = maxLineGap)


avg_lines = average(img_original, lines)
"""


def extractLinesFromImage(img):
    img_original = img.copy()

    mean = cv2.mean(img)
    print("mean: {}".format(mean)) 
    imgMean = (mean[0] + mean[1] + mean[2]) / 3; 


    # contrast increase for low light
    if (imgMean < 127): 
        contrast = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
        grey = cv2.cvtColor(contrast, cv2.COLOR_BGR2GRAY)
    else: 
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # gaussian blur 
    # (11, 11) kernel dimensions blur it such that it eliminates other slow changing noise (contrast indicator)
    gauss = cv2.GaussianBlur(grey, (3, 3), 0)

    # canny edge detection 
    edges = cv2.Canny(gauss, 55, 95)

    # masked image 
    space = region(edges)

    # get Hough lines + average
    lines = cv2.HoughLinesP(space, resolutionRho, resolutionTheta, threshold, minLineLength = minLineLength, maxLineGap = maxLineGap)


    avg_lines = average(img_original, lines)

    out = applyLinesToImage(img_original, avg_lines)

    return (avg_lines, out)


images = ["./src/sandbox/cv/detectLines/small/aldrich3.png"]

for im in images:
    avg_lines, out = extractLinesFromImage(im)



# ##########################################################
# out = applyLinesToImage(img_original, avg_lines)
# display(out)