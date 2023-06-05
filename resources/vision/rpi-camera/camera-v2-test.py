'''
HOW TO USE the Raspberry Pi Camera Module, 2019. 
https://www.youtube.com/watch?v=VzYGDq0D1mw.
'''

from picamera import PiCamera
from time import sleep

camera = PiCamera()
camera.start_preview(alpha=192)
sleep(1)
camera.capture("/home/lenny/dojo/internship2023/resources/vision/rpi-camera/cameraV2-test.jpg")
camera.stop_preview()