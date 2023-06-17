import serial
import time


# Setup Serial Communication
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()

# Camera Inference Function


def camera_inference():
    x, y, z = 10, 20, 30

    return (x, y, z)


def arm_comms_simulator(gpio_pick: int = 0, gpio_place: int = 0):
    # * Pi - Arduino Arm Communication
    # 1. Pi polls GPIO for event from Mobile-Platform
    # 2. If GPIO.Pick is detected, run camera_inference function
    # 3. Get coordinates from camera_inference function
    # 4. Pass the action + coordinates via serial to Arm
    # 5. Arm executes action -> Await message from Arm
    # 6. If message is SUCCESS, send GPIO.Go to Mobile-Platform
    #! 7. If message is FAIL, send GPIO.Fail to Mobile-Platform
    # 8. Mobile-Platform pulls GPIO.Pick low
    # ? 9. Pi pulls GPIO.Go low (for next cycle)

    while True:
        # 2. If GPIO.Pick is detected, run camera_inference function
        if gpio_pick == 1:
            print("GPIO.Pick detected")

            # 3. Get coordinates from camera_inference function
            coordinate_string = camera_inference()
            print(f"coordinate_string: {coordinate_string}")

            # 4. Pass the action + coordinates via serial to Arm
            action = 0  # 0 = Pick, 1 = Place

            # Format message to send to Arm
            formatted_msg = f"{action}|{coordinate_string[0]}|{coordinate_string[1]}|{coordinate_string[2]}\n"
            print(f"Sending formatted_msg: {formatted_msg}")

            # Send message to Arm
            ser.write(formatted_msg.encode('utf-8'))
            # 5. Arm executes action -> Await message from Arm

        # 5. Arm executes action -> # Await message from Arm
        arm_msg = ser.readline().decode('utf-8').rstrip()
        if arm_msg:
            print(f"Received arm_msg: {arm_msg}")
        else:
            print("Waiting for Arm message")

        # ? Await message from Arm
        # while not arm_msg:
        #     print("Waiting for Arm message")
        #     arm_msg = ser.readline().decode('utf-8').rstrip()

        # print(f"Received arm_msg: {arm_msg}")

        # 6. If message is SUCCESS, send GPIO.Go to Mobile-Platform
        if arm_msg == "SUCCESS":

            print("Mobile Platform Go")
            # 8. Mobile-Platform pulls GPIO.Pick low
            # gpio_pick = 0

            return 1

            # ? Run Loop again
            # break

        # time.sleep(1)


arm_comms_simulator(1)
