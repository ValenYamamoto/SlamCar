import cv2
import matplotlib.pyplot as plt
import numpy as np
from numpy.core.fromnumeric import put

windowName = 'image'

# Hough line parameters 
# rho: distance from (0,0) top-left corner of image 
# theta: angle of that line from top-axis 
# minLineLength: 
# maxLineGap: 

alpha = 1.5; 
beta = 1; 

resolutionRho = 1
resolutionTheta = np.pi/180
threshold = 10
minLineLength = 10
maxLineGap = 200

resolutionRhoRange = (1, 4)
resolutionThetaRange = (np.pi/180, 10)
thresholdRange = (0, 200)
minLineLengthRange = (0, 100)
maxLineGapRange = (0, 200)
gaussBlurRange = (5, 101)

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
    y2 = int(y1 * (3/5))
    x1 = int((y1 - y_int) // slope )
    x2 = int((y2 - y_int) // slope )
    return np.array([x1, y1, x2, y2])

def average(img, lines): 
    left = []
    right = []
    for line in lines: 
        print(line)
        x1, y1, x2, y2  = np.reshape(line, 4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_int = parameters[1]
        if slope < 0: 
            left.append((slope, y_int))
        else: 
            right.append((slope, y_int))
    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)
    left_line = make_points(img, left_avg)
    right_line = make_points(img, right_avg)
    return np.array([left_line, right_line])
    
def applyLinesToImage(img, lines):
    cpy = img.copy()
    for line in lines:
        x1, y1, x2, y2 = line
        cv2.line(cpy, (x1, y1), (x2, y2), (255, 0, 0), 3)
    return cpy

def displayLines(image, lines):
 lines_image = np.zeros_like(image)
 if lines is not None:
   for line in lines:
     x1, y1, x2, y2 = line
     cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
 return lines_image

# import unadulterated image 
img = cv2.imread("./sandbox/cv/detectLines/small/aldrich0.png", cv2.IMREAD_UNCHANGED)
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
gauss = cv2.GaussianBlur(grey, (1, 1), 0)

# canny edge detection 
edges = cv2.Canny(gauss, 40, 60)

# masked image 
space = region(edges)

# get Hough lines + average
lines = cv2.HoughLinesP(space, resolutionRho, resolutionTheta, threshold, minLineLength = minLineLength, maxLineGap = maxLineGap)
avg_lines = average(img, lines)
out = displayLines(img_original, avg_lines)
display(out)





#avg_lines = average(img_original, lines)
#black_lines = displayLines(img_original, lines)

#cv2.imshow("lanes", black_lines)

#lanes = cv2.addWeighted(img_original, 0.8, black_lines, 5, 5)
##cv2.imshow("lanes", lanes)
#cv2.waitKey(0)

# should do some averaging here to get clean, single lines
# out = displayLines(img_original, avg_lines)
# display(out)





# out = applyLinesToImage(img_original, lines)
# display(out)



