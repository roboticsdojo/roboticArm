from imutils.video import VideoStream
import cv2
import time


print("[INFO] starting video stream...")
vs = VideoStream(src=0).start() 
time.sleep(2.0)

while True:
    frame = vs.read()
    print(frame)
    
    if not frame:
        break

cv2.destroyAllWindows()
vs.stop()