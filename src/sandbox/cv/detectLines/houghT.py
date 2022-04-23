from curses import window
import cv2
from cv2 import HoughLinesP
import matplotlib.pyplot as plt
import numpy as np
import laneDetect
import time
resolutionRho=2.26
#resolutionTheta=0.01745277778
resolutionTheta=0.00162 #more processing thoq
threshold=250
rectTopMaskHeight=315


def printSettings():
    print("resolutionRho="+str(resolutionRho))
    print("resolutionTheta="+str(resolutionTheta))
    print("threshold="+str(threshold))
    print("rectTopMaskHeight="+str(rectTopMaskHeight))




windowName = "Hello"
resolutionRhoRange = (1, 1000)
resolutionThetaRange = (1, 3500)
thresholdRange = (200, 400)
rectTopMaskHeightRange = (0, 1080)

topMaskRange = (0, 1080)


def updateResolutionRho(val):
    print("actual" + str(val))
    global resolutionRho
    resolutionRho = val/100
    print(resolutionRho)

    #updateImage()

def updateResolutionTheta(val):
    global resolutionTheta
    resolutionTheta = val/100000
    print(str(resolutionTheta))
    #updateImage()

def updateThreshold(val):
    print(val)
    global threshold 
    threshold = val
    #updateImage()


def updateRectTopMaskHeight(val):
    print(val)
    global rectTopMaskHeight
    rectTopMaskHeight = val
    #updateImage()


def updateImage():
    cv2.imshow(windowName, model())


def drawSwag(img, lines):
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
        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

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
    # for rho,theta in lines[0]:
    #     a = np.cos(theta)
    #     b = np.sin(theta)
    #     x0 = a*rho
    #     y0 = b*rho
    #     x1 = int(x0 + 1000*(-b))
    #     y1 = int(y0 + 1000*(a))
    #     x2 = int(x0 - 1000*(-b))
    #     y2 = int(y0 - 1000*(a))

    # cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

def model(img):
    start = time.time()

    global resolutionRho, resolutionTheta, threshold, rectTopMaskHeight
    #global img,edges
    edges = img.copy()




    edges = laneDetect.applyGaussianBlur(edges, 3)

    edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)


    edges = cv2.Canny(edges, 400,650)

    edges = laneDetect.rectTopDownMask(edges, rectTopMaskHeight)

    #edges = laneDetect.getHoughLinesP(edges, resolutionRho, resolutionTheta, threshold, mll=minLineLength, mlg=maxLineGap)
    #edges = cv2.HoughLinesP(edges)
    #edges = cv2.HoughLinesP(edges, 1, 3.1415/180, 50, 50, 10)

    lines = cv2.HoughLines(edges,1,np.pi/180,200)

    lines = cv2.HoughLines(edges,resolutionRho,resolutionTheta,threshold)

    out = img.copy()
    if(np.any(lines)):
        lines = convertPolarToCartesian(lines)

        

        #laneDetect.drawHoughLines2(out, lines)

        lanes = laneDetect.average(out, lines)

        

        
        



        data = laneDetect.findSteeringAngle(out, lanes, drawMidLines=True)
        elapsed = time.time() - start

        elapsed = elapsed * 1000
        
        cv2.putText(out, "ms: " + str(round(elapsed, 3)), (50, 140), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

        laneDetect.drawAverageLines(out, lanes)

        laneDetect.displayHeadingLine(out, data[0])

    


    


    return out

def playVid():

    cap = cv2.VideoCapture("src/sandbox/cv/detectLines/small/video.mp4")


    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    # Read until video is completed
    while(cap.isOpened()):
    # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
            # Display the resulting frame
            frame = model(frame)
            cv2.imshow(windowName,frame)

            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                exit()

    # Break the loop
        else: 
            break

    # When everything done, release the video capture object
    cap.release()

    # Closes all the frames
    #cv2.destroyAllWindows()

cv2.namedWindow(windowName)
cv2.createTrackbar('HoughThreshold', windowName, thresholdRange[0], thresholdRange[1],  updateThreshold)
cv2.createTrackbar('resolutionRho', windowName, resolutionRhoRange[0], resolutionRhoRange[1], updateResolutionRho)
cv2.createTrackbar('resolutionTheta', windowName, resolutionThetaRange[0], resolutionThetaRange[1], updateResolutionTheta)
cv2.createTrackbar('maskTop', windowName, rectTopMaskHeightRange[0], rectTopMaskHeightRange[1], updateRectTopMaskHeight)

print("Starting Params:")
printSettings()

while 1:
    playVid()
# cv2.imshow(windowName, model())
# cv2.waitKey(0)

# # cv2.waitKey(0)
# cv2.destroyAllWindows
# exit()
