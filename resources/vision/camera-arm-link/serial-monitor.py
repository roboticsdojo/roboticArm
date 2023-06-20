import serial
import time

# Setup Serial Communication
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
ser.reset_input_buffer()


while True:
    # if ser.in_waiting > 0:
    #     ser_msg = ser.readline().decode('utf-8').rstrip()

    #     print(ser_msg)
    if ser.readline():
        ser_msg = ser.readline().decode('utf-8').rstrip()
        print(ser_msg)

        # print("\n")
