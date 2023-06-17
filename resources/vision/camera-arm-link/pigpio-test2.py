# import the GPIO and time package
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)

# loop through 50 times, on/off for 1 second
for i in range(50):
    print("LED on")
    GPIO.output(23, True)
    time.sleep(1)
    GPIO.output(23, False)
    print("LED off")
    time.sleep(1)
GPIO.cleanup()
