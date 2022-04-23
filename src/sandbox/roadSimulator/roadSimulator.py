#include <opencv2/videoio.hpp>
# pip install ffmpeg-python


# ---------------------------------------------
# Author(s): Sharon Xia 
# Written for: UCI ECE Senior Design 2021-2022
# Team: Roombot
# ---------------------------------------------

import numpy as np
import cv2 as cv
from cv2 import CAP_FFMPEG

import keyboard
from pynput.keyboard import Key, Listener

import threading # https://realpython.com/intro-to-python-threading/
import time

import os


VIDEO_DIMENSIONS = (1700, 800) # Desired max-dimensions for video (w, h)

# Keyboard Control Increments
SIZE_INCREMENT = 100
MOVE_INCREMENT = 50


# ---------------------------------------------
# ROADSIMULATOR PIPELINE: 
# ---------------------------------------------
# Original Video Frame
# -> Cropped Video Frame
# -> Resized to VIDEO_DIMENSIONS global var
# -> Frame is displayed as part of video
# --------------------------------------------
# Other info: 
# --------------------------------------------
# class runs one video at a time
# control simulator through keyboard input (w, a, s, d, [,<], [.>])
# - more info in @__keyboardVideoControls(self)
class RoadSimulator:

  def __init__(self, videoFile = None):
    self.videoFile = ""
    self.cap = None
    self.videoDimensions = (0, 0) # stores current video max size (w, h)

    # initialize video display settings -- rewritten in future
    self.cropWidth = 200
    self.cropHeight = 150
    self.cropPosX = 100
    self.cropPosY = 100

    # for quitting video early
    self.stopVideo = False

    if videoFile != None: 
      self.setVideo(videoFile)

  # retrieval methods
  def getVideoCapture(self):
    return self.cap

  def getVideoFilePath(self):
    return self.videoFile

  # opens different video file
  #   can use files taken from @getVideoLocations()
  #   requires file's absolute path + file type (.mp4, .avi,..etc)
  # also updates crop display settings according to video size
  def setVideo(self, videoFile):
    self.videoFile = videoFile
    
    cap = cv.VideoCapture(videoFile)
    self.cap = cap
    
    # check video opened
    if not cap.isOpened():
      print("Error opening the video file")
      return

    self.__updateCropDimensionsToVideo()


  # prints frame rate, frame count, and video_time
  def printVideoInfo(self):
    divider = "========================"

    if (self.cap == None):
      print(divider)
      print("No video set")
      print(divider)
      return

    print(divider)

    fps = int(self.cap.get(cv.CAP_PROP_FPS))
    frame_number = self.cap.get(cv.CAP_PROP_FRAME_COUNT)
    print("Frame Rate : ",fps,"frames per second") 
    print("Frame count : ", frame_number)

    if (fps > 0):
      video_time = frame_number/fps
      print(video_time)

    print(divider)

  
  # resize frame for __runVideo to fit inside VIDEO_DIMENSIONS
  # returns resized frame
  def __resizeFrame(self, frame):

    currentHeight = self.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
    currentWidth = self.cap.get(cv.CAP_PROP_FRAME_WIDTH)
    currentHtoW = currentHeight/currentWidth

    desiredWidth = VIDEO_DIMENSIONS[0]
    desiredHeight = VIDEO_DIMENSIONS[1]
    desiredHtoW = desiredHeight/desiredWidth

    height = currentHeight
    width = currentWidth
    
    if currentHtoW > desiredHtoW:
      height = desiredHeight
      width = (int)(height/currentHtoW)

    else:
      width = desiredWidth
      height = (int)(width * currentHtoW)

    #print(str((currentHeight, currentWidth)) + " -> " + str((height, width)))

    return cv.resize(frame, (width, height))


  # updates crop dimensions according to video size
  def __updateCropDimensionsToVideo(self):
    if self.cap == None:
      print("Cannot update crop dimensions: No VideoCapture stored")
      return

    self.cropHeight = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    self.cropWidth = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))

    self.videoDimensions = (self.cropWidth, self.cropHeight) 

    self.cropPosX = 0
    self.cropPosY = 0


  # by user controls
  # - positionUpdate = (x: int, y: int)
  # - sizeUpdate = int [based on larger dimension:  width/height]
  def __updateCropDimensions(self, positionUpdate, sizeUpdate):

    # returns adjusted value between min and max
    def limitDimension(x, minimum, maximum):
      if (minimum > maximum):
        print("invalid dimension limit: minimum > maximum")
        return x

      x = max(minimum, x)
      x = min(maximum, x)

      return x
    
    # update pos first
    self.cropPosX += positionUpdate[0]
    self.cropPosY += positionUpdate[1]


    # update size after
    if sizeUpdate != 0:
      currentVideoWtoH = (self.videoDimensions[0]/self.videoDimensions[1])

      if currentVideoWtoH <= 1: # adjust height to sizeUpdate
        self.cropHeight += sizeUpdate 
        self.cropHeight = limitDimension(self.cropHeight, 0, self.videoDimensions[1]) # ensure size is valid

        self.cropWidth = int(self.cropHeight * currentVideoWtoH) # scale
        self.cropWidth = limitDimension(self.cropWidth, 0, self.videoDimensions[0])
      
      else: # adjust width to sizeUpdate
        self.cropWidth += sizeUpdate 
        self.cropWidth = limitDimension(self.cropWidth, 0, self.videoDimensions[0]) # ensure size is valid

        self.cropHeight = int(self.cropWidth / currentVideoWtoH) # scale
        self.cropHeight = limitDimension(self.cropHeight, 0, self.videoDimensions[1])
    

    # ensure cropPos is valid
    self.cropPosX = limitDimension(self.cropPosX, 0, self.videoDimensions[0] - self.cropWidth)
    self.cropPosY = limitDimension(self.cropPosY, 0, self.videoDimensions[0] - self.cropHeight)


  # subfunction for __runVideo
  def __getCropDimensions(self):

    h1 = self.cropPosY
    h2 = self.cropPosY + self.cropHeight
    w1 = self.cropPosX
    w2 = self.cropPosX + self.cropWidth

    return (h1, h2, w1, w2)


  # function called by @runSimulator()
  # run through video + display
  def __runVideo(self):
    while(self.cap.isOpened() and not self.stopVideo):
      # vCapture.read() methods returns a tuple, first element is a bool
      # and the second is frame

      ret, frame = self.cap.read()

      if not ret:
        break

      # crop video image [h1:h2, w1:w2]
      cd = self.__getCropDimensions()
      # print("crop dimensions: " + str(cd))
      frame = frame[cd[0]:cd[1], cd[2]:cd[3]]

      # resize to VIDEO_DIMENSIONS
      frame = self.__resizeFrame(frame)

      cv.imshow('Frame',frame)
      k = cv.waitKey(20)

    # When everything done, release the capture
    self.stopVideo = False
    self.cap.release()
    cv.destroyAllWindows()


  # function called by @runSimulator()
  # controls video display through keyboard
  # - w, a, s, d keys to move video side to side or up/down
  # - [,<] [.>] keys to zoom in or zoom out
  # - q to stop video before it's done
  def __keyboardVideoControls(self):

    while self.cap.isOpened():
      positionUpdate = [0, 0] # (x: int, y: int)
      sizeUpdate = 0

      print(keyboard.read_key())

      match (keyboard.read_key()):
        case 'w':
          positionUpdate[1] -= MOVE_INCREMENT
        case 'a':
          positionUpdate[0] -= MOVE_INCREMENT
        case 's': 
          positionUpdate[1] += MOVE_INCREMENT
        case 'd':
          positionUpdate[0] += MOVE_INCREMENT
        case ',':
          sizeUpdate -= SIZE_INCREMENT
        case '.':
          sizeUpdate += SIZE_INCREMENT
        case 'q':
          self.stopVideo = True


      if (positionUpdate != [0,0] or sizeUpdate != 0):
        print("position update: " + str(positionUpdate))
        print("size update: " + str(sizeUpdate))

      self.__updateCropDimensions(positionUpdate, sizeUpdate)
      time.sleep(0.1)

    return

    
  def runSimulator(self):
    # TODO put threads together
    keyboardControlThread = threading.Thread(target=self.__keyboardVideoControls, args=())
    videoThread = threading.Thread(target=self.__runVideo, args=())

    keyboardControlThread.start()
    videoThread.start()

    return


  @staticmethod
  def getVideoLocations():
    videoLocation = "C:/Users/longl/OneDrive/2022 Winter/SDP/sdpf10/src/sandbox/roadSimulator/videos"
    os.chdir(videoLocation) # necessary for os.path.abspath to create right path

    return [os.path.abspath(file) for file in os.listdir()]



    
files = RoadSimulator.getVideoLocations()

videoFile = files[1]
print(files)

simulator = RoadSimulator(videoFile)
simulator.printVideoInfo()
simulator.runSimulator()







