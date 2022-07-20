from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# Ubuntu -> cv2.videoCapture(0) ?

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.rotation = 180
camera.resolution = (1280, 720) # 720p, (640, 480)
rawCapture = PiRGBArray(camera)

# allow the camera to warmup
time.sleep(2)

# grab an image from the camera
camera.capture(rawCapture, format="bgr")
image = rawCapture.array

# save opencv image
cv2.imwrite("./image.jpg", image)
