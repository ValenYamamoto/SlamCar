# WORKING LANE DETECTION ALG #

import cv2
import matplotlib.pyplot as plt
import numpy as np
import time



windowName = 'image'

alpha=0.6
beta=0.4
contrastAdjustThreshold=127
resolutionRho=2.26
resolutionTheta=0.00162 #more processing thoq
threshold=250
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

def rectTopDownMask(img, h): 
    height = img.shape[0]
    width = img.shape[1]
    #rect = np.array([[(0, 0), (width, 0), (0, rectHeight), (width, rectHeight)]])
    rect = np.array([[(0, height), (0, height - h), (width, height - h), (width, height)]])
    mask = np.zeros_like(img)
    mask = cv2.fillPoly(mask, rect, 255)
    mask = cv2.bitwise_and(img, mask)
    return mask 

def rectBottomUpMask(img, h):
    height = img.shape[0]
    width = img.shape[1]
    #rect = np.array([[(0, 0), (width, 0), (0, rectHeight), (width, rectHeight)]])

    rect = np.array([[(0, h), (0, 0), (width, 0), (width, h)]])

    # top left, bottom left, bottom, right, top  right
    mask = np.zeros_like(img)
    mask = cv2.fillPoly(mask, rect, 255)
    mask = cv2.bitwise_and(img, mask)
    return mask 

def average(img, lines): 

    def make_points(img, average): 
        #print("average" + str(average))
        slope, y_int = average 
        y1 = img.shape[0]
        y2 = int(y1 * (1/4))
        x1 = int((y1 - y_int) // slope )
        x2 = int((y2 - y_int) // slope )
        return np.array([x1, y1, x2, y2])

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
    left_line = None
    right_line = None
    if(len(left)):
        left_avg = np.average(left, axis=0)
        left_line = make_points(img, left_avg)

    if(len(right)):
        right_avg = np.average(right, axis=0)
        right_line = make_points(img, right_avg)

    return [left_line, right_line]



def convertPolarToCartesian(lines):
    rtnLines = []
    for line in lines:
        len = 2000
        rho,theta = line[0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + len*(-b))
        y1 = int(y0 + len*(a))
        x2 = int(x0 - len*(-b))
        y2 = int(y0 - len*(a))
        rtnLines.append((x1, y1, x2, y2))
        #cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    return rtnLines

def drawHoughLines2(img, edges):
    if(np.any(edges)):
            for temp in edges:
                l = temp
                img = cv2.line(img, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 5)
                


def drawHoughLines(img, edges):
    if(np.any(edges)):
            for temp in edges:
                l = temp[0]
                img = cv2.line(img, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 5)

    return img
                #cpyColor = cv2.line(cpyColor, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 5)

def drawAverageLines(img, avg):
    if(np.any(avg[0])):
        cv2.line(img, (avg[0][0], avg[0][1]), (avg[0][2], avg[0][3]), (0, 255, 0), 15)
    if(np.any(avg[1])):
        cv2.line(img, (avg[1][0], avg[1][1]), (avg[1][2], avg[1][3]), (0, 255, 0), 15)


def applyLinesToImage(img, lines):
#    cpy = img.copy()
    if(lines):
        if(np.any(lines[0])):
            x1, y1, x2, y2 = lines[0]
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 15)
        else:
            print("No lines to draw for right")
        if(np.any(lines[1])):
            x1, y1, x2, y2 = lines[1]
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 15)
        else:
            print("No Lines to draw for left")
    return img

def applyGscale(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

def modelTest1(img):
    alpha=0.6
    beta=0.4
    contrastAdjustThreshold=127
    resolutionRho=2.26
    resolutionTheta=0.00162 #more processing thoq
    threshold=250
    ksize=3
    cannyFirstThreshold=67
    cannySecondThreshold=88
    rectTopMaskHeight=153
    rectBottomMaskHeight=640
    cpy = img.copy()

    cpy = applyContrastAdjustment(cpy, alpha, beta, contrastAdjustThreshold)
    
    cpy = applyGscale(cpy)

    cpy = applyGaussianBlur(cpy, ksize)

    edges = getCannyEdges(cpy, cannyFirstThreshold, cannySecondThreshold)
    cpy = edges

    cpy = rectTopDownMask(cpy, rectTopMaskHeight)
    cpy = rectBottomUpMask(cpy, rectBottomMaskHeight)

    edges = getHoughLinesP(cpy, resolutionRho, resolutionTheta, threshold, mll=minLineLength, mlg=maxLineGap)
    #cpyColor = drawHoughLines(cpyColor, edges)
    if(not np.any(edges)):
        return None

    # average lines after doHough1
    avg_lines = average(img, edges)

    drawAverageLines(img, avg_lines)

    rtn = findSteeringAngle(img, avg_lines=avg_lines, drawMidLines=True)
    #drawMidPointLine(img, 400)
    #cv2.putText(img, "OOGABOOGA", (5,40), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
    if(rtn[1]):
        cv2.line(img, (rtn[1], 300), (rtn[1], 480), (0, 150, 255), 8)

    return rtn[0]


def modelTest2(img):
    alpha=0.6
    beta=0.4
    contrastAdjustThreshold=127
    resolutionRho=2.26
    resolutionTheta=0.00162 #more processing thoq
    threshold=250
    ksize=3
    cannyFirstThreshold=67
    cannySecondThreshold=88
    rectTopMaskHeight=153
    rectBottomMaskHeight=640

    alpha=0.6
    beta=0.4
    contrastAdjustThreshold=127
    resolutionRho=2.26
    resolutionTheta=0.00162 #more processing thoq
    threshold=250
    ksize=3
    cannyFirstThreshold=67
    cannySecondThreshold=88
    rectTopMaskHeight=153
    rectBottomMaskHeight=640

    start = time.time()

    edges = img.copy() 

    edges = applyGaussianBlur(edges, 3)

    edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(edges, 400,650)

    lines = cv2.HoughLines(edges,resolutionRho,resolutionTheta,threshold)

    out = img

    if(np.any(lines)):
        lines = convertPolarToCartesian(lines)

        drawHoughLines2(out, lines)

        lanes = average(out, lines)

        data = findSteeringAngle(out, lanes, drawMidLines=True)

        elapsed = time.time() - start

        elapsed = elapsed * 1000

        cv2.putText(out, "ms: " + str(round(elapsed, 3)), (50, 140), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

        drawAverageLines(out, lanes)

        displayHeadingLine(out, data[0])

        return data[0]
    
    else:
        return None


def applyContrastAdjustment(img, a, b, doContrastThreshold):
    mean = cv2.mean(img)
    imgMean = (mean[0] + mean[1] + mean[2]) / 3; 

    # contrast increase for low light
    grey = None
    if (imgMean < doContrastThreshold): 
        img = cv2.convertScaleAbs(img, alpha=a, beta=b)
    
    return img


def applyGaussianBlur(img, ksize):
    return cv2.GaussianBlur(img, (ksize, ksize), 0)

def getCannyEdges(img, thresh1=55, thresh2=95): #TODO: parameterize
    return cv2.Canny(img, thresh1, thresh2, apertureSize=3)

def getHoughLinesP(img, resolutionRho, resolutionTheta, thresh, mll, mlg):
    return cv2.HoughLinesP(img, resolutionRho, resolutionTheta, thresh, minLineLength=mll, maxLineGap=mlg)

def getHoughlines(img, resolutionRho, resolutionTheta, thresh):
    return cv2.HoughLines(img, resolutionRho, resolutionTheta, thresh)

#Assume robot starts at center of road
def getXMidPointFromAvgLines(img, avg_lines):
    #y2-y1/x2-x1

    left = avg_lines[0]
    right = avg_lines[1]
    m_left = (left[3] - left[1])/(left[2]-left[0])
    m_right = (right[3] - right[1])/(right[2]-right[0])

    #TODO: idk why i had to do this but in desmos this looked right
    #img_h =  480#TODO: do not hardcode 
    img_h = img.shape[0]


    m_left = m_left * -1
    m_right = m_right * -1
    
    #Sample x values for y=edge of image
    #TODO: do not hardcode
    xLeftAtBottomEdge = ((img_h - left[1])/m_left) + left[0]
    xRightAtBottomEdge = ((img_h - right[1])/m_right) + right[0]

    mid = ((xRightAtBottomEdge - xLeftAtBottomEdge)/2) + xLeftAtBottomEdge

    offset = 0

    mid = mid + offset

    return mid

#Given a mid point this returns a proportional steering angle in range (0,180)
def pushingPLoop(mid):
    #hardcoded mid offset
    mid_offset = 0
    mid = mid + mid_offset

    steering_range = 180

    # mid_max = 390
    # mid_min = 250

    mid_max = 960+200
    mid_min = 960-200

    if(mid > mid_max):
        mid = mid_max

    if(mid < mid_min):
        mid = mid_min
    

    mid_zeroed = mid - mid_min
    pushing_p = steering_range/(mid_max - mid_min)


    angle = pushing_p * mid_zeroed

    return angle
    


def drawMidPointLine(img, mid):
    #TODO do not hardcode image dimmensions
    # h = 480
    # w = 640
 
# height, width, number of channels in image
    h = img.shape[0]
    w = img.shape[1]
    mid = int(mid)
    mid_line_length = 40
    cv2.line(img, (mid, h), (mid, h-mid_line_length), (0, 150, 255), 8)
    cv2.line(img, (int(w/2), h), (int(w/2), (h-mid_line_length) - 100), (255, 150, 255), 4)

# function for finding steering angle alone 
def findSteeringAngle(img, avg_lines, drawMidLines=False): 
    # parameters needed to find offset: image height, width, and horizontal mid point 
    height = img.shape[0]
    width = img.shape[1]
    mid = int(width/2)

    # compute x_offset, or deviation from center line
    # if x_offset < 0, steer left and if x_offset > 0, steer right 

    # TWO LINE CASE, both avg_lines[0] and avg_lines[1] (left and right) exist 
    if(np.any(avg_lines[0]) and np.any(avg_lines[1])): 
        #Kellys Logic
        # left_x2 = avg_lines[0][2]
        # right_x2 = avg_lines[1][2]
        # x_offset = int((left_x2 + right_x2) / 2 - mid)
        # y_offset = int(height/2)

        #Jonathan Edit
        mid = getXMidPointFromAvgLines(img, avg_lines=avg_lines)
        mid = int(mid)
        if drawMidLines and mid != None:
           drawMidPointLine(img, mid)
        
        angle = pushingPLoop(mid)
        return [angle, mid]
        


    # ONE LINE CASE
    elif(np.any(avg_lines[0]) and not np.any(avg_lines[1])): 
        x1 = avg_lines[0][0]
        x2 = avg_lines[0][2]
        x_offset = x2 - x1 
        y_offset = int(height/2)
    elif(np.any(avg_lines[1]) and not np.any(avg_lines[0])): 
        x1 = avg_lines[1][0]
        x2 = avg_lines[1][2]
        x_offset = x2 - x1 
        y_offset = int(height/2)

    # if no lines detected, return nothing 
    else: 
        return None
        print("No Lines Detected")

    # compute angle between heading vector and center line 
    angle_to_mid_rad = np.arctan(x_offset/y_offset)
    angle_to_mid_deg = int(angle_to_mid_rad * 180 / np.pi)

    # compute steering angle: input to servo to steer based on lane lines 
    steering_angle = 90 - angle_to_mid_deg  

    return [steering_angle, None]

# function for displaying heading vector based on calculated steering angle 
def displayHeadingLine(img, steering_angle):    

    if(steering_angle == None): 
        return img
    else:
        height = img.shape[0]
        width = img.shape[1]

        steering_angle = 180 - steering_angle
        # compute angle between heading vector and center line 
        angle_to_mid_deg = 90 - steering_angle

        # compute steering angle for heading vector display 
        steering_angle_display = angle_to_mid_deg + 90 
        steering_angle_rad = steering_angle_display / 180 * np.pi 

        # compute points needed to display heading vector line 
        x1 = int(width/2)
        y1 = height
        x2 = int(x1 - height / 2 / np.tan(steering_angle_rad))
        y2 = int(height/2)

        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.putText(img, "Angle: " + str(round(steering_angle, 2)), (50, 100), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 5)
