import cv2
import os
import numpy as np

from picamera import PiCamera
from time import sleep

from datetime import datetime

def compareImages(img1, img2):
  blurredImg1 = cv2.GaussianBlur(img1, (5,5), 0)
  blurredImg2 = cv2.GaussianBlur(img2, (5,5), 0)

  diffImage = cv2.subtract(blurredImg1, blurredImg2)

  #  Generate Gaussian noise
  gauss = np.random.normal(0,1,diffImage.size)
  gauss = gauss.reshape(diffImage.shape[0], diffImage.shape[1], diffImage.shape[2]).astype('uint8')
  
  blurredGauss = cv2.GaussianBlur(gauss, (5,5), 0)
  
  # Subtract the Gaussian noise to the image
  diffImage = cv2.subtract(diffImage, blurredGauss)

  meanPixelValue = cv2.meanStdDev(diffImage)

  diffMean = sum(meanPixelValue[0])/len(meanPixelValue[0])
  diffStdDev = sum(meanPixelValue[1])/len(meanPixelValue[1])
  
  averagePixelValues.append(diffMean)
  averageStdDevValues.append(diffStdDev)
  
  global threshold = (sum(averagePixelValues)/len(averagePixelValues)) + (sum(averageStdDevValues)/len(averageStdDevValues))
  print("Current: "+str(diffMean))
  print("Average: "+str(threshold))
  
  if(diffMean > threshold and numIterations > initialCalibrationPeriod):
    del averagePixelValues[-1]
    del averageStdDevValue[-1]
    print("Motion detected!!")
    return True

  return False

def detectMotion(img1Location, img2Location):
  img1 = cv2.imread(img1Location)
  img2 = cv2.imread(img2Location)
  
  if(compareImages(img1, img2)):
    return True
  else:
    return False
   
def recordVideo(camera, videoDuration):
  try:
    now = datetime.now()
    currentTime = now.strftime("%H_%M_%S")
    videoName = "motion_"+currentTime+".h264"
    camera.start_recording("motion_"+currentTime+".h264")
    sleep(videoDuration)
    camera.stop_recording()

    #os.remove(videoName)
    return True
  except Exception as e:
    print(e)
    return False

def monitorLoop(camera, imageDelay, videoDuration):
  while(os.path.exists("active")):
    sleep(imageDelay)
    camera.capture("image_before.jpg")
    if(os.path.exists("image_before.jpg") and os.path.exists("image_after.jpg")):
      if(detectMotion("image_before.jpg", "image_after.jpg")):
        recordVideo(camera, videoDuration)
      os.remove("image_after.jpg")
    
    sleep(imageDelay)
    camera.capture("image_after.jpg")
    if(os.path.exists("image_before.jpg") and os.path.exists("image_after.jpg")):
      if(detectMotion("image_before.jpg", "image_after.jpg")):
        recordVideo(camera, videoDuration)

    os.remove("image_before.jpg")
    global numIterations = numIterations + 1
    
  if(os.path.exists("image_after.jpg")):
    os.remove("image_after.jpg")
  
  if(os.path.exists("image_before.jpg")):
    os.remove("image_before.jpg")
    


if __name__ == "__main__":
  initialThreshold = 5

  initialCalibrationPeriod = 100
  numIterations = 0

  threshold = initialThreshold

  averagePixelValues = []
  averageStdDevValues = []

  camera = PiCamera()
  monitorLoop(camera, 2, 10)