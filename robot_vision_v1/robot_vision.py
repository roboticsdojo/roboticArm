from ultralytics.yolo.utils.plotting import Annotator
from model_inference import infer, model
import RPi.GPIO as GPIO
from datetime import datetime
import time
import cv2


# Setup Camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
# Set Dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)


# Setup GPIO
pick_pin = 23
place_pin = 24
go_pin = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(pick_pin, GPIO.IN)
GPIO.setup(place_pin, GPIO.IN)
GPIO.setup(go_pin, GPIO.OUT)


mobile_platform_event = 0


# ---------- Camera Related Functions ----------
def camera_inference():
    x, y, z = 10, 20, 30

    return (x, y, z)


def video_snap_infer():
    while cap.isOpened():
        now = datetime.now()

        ret, frame = cap.read()
        if not ret:
            print("Failed to Read Camera Frame")
            break

        # cv2.imwrite('infer_from_snapshot.jpg', frame)

        # Get FPS
        FPS = cap.get(cv2.CAP_PROP_FPS)

        # Show Feed
        live_window = 'Live Feed'
        # Re-position Window
        cv2.namedWindow(live_window)
        cv2.moveWindow(live_window, 0, 0)
        cv2.imshow(live_window, frame)

        if cv2.waitKey(1) & 0xFF == ord('l'):
            # Infer on snapshot
            print(f"[{now}]> Infer on snapshot")
            inference_result = infer(frame)

            print("\n-------------------------\n")
            print(inference_result[0].boxes)
            print("\n-------------------------\n")

            # Visualize Result
            # https://stackoverflow.com/questions/75324341/yolov8-get-predicted-bounding-box
            for r in inference_result:
                print(f"r: {r}")

                annotator = Annotator(frame)

                boxes = r.boxes
                print(f"boxes: {boxes}")
                for box in boxes:
                    print(f"box: {box}")

                    # get box coordinates in (top, left, bottom, right) format
                    b = box.xyxy[0]
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])
                    print(f"Box: {b}, Class: {c}")

            inference_frame = annotator.result()
            inference_win = 'YOLO V8 Inference Result'
            # Re-position Window
            cv2.namedWindow(inference_win)
            cv2.moveWindow(inference_win, 640, 0)
            cv2.imshow(inference_win, inference_frame)

            # * Log Results
            if inference_result:
                # save visualization
                # ? Manually draw bounding boxes and save those for visualization
                # ? Realtime video feed for debugging.
                # inference_result.save(f"./logs/images/{now}.jpg")
                cv2.imwrite(f'./logs/images/{now} None.jpg', frame)
                print(f"[{now}]> Save result as image [SUCCESS]")
            else:
                print(f"[{now}]> No prediction")
                # Save frame
                cv2.imwrite(f'./logs/images/{now} None.jpg', frame)

            #! failed
            # ? json
            inference_result[0].save_txt(f"./logs/labels/results.txt")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


video_snap_infer()

# * Super Loop
# while True:

#     # * Mobile-Platform - Pi Communication Simulator
#     # 1. Pi Polls GPIO for event from Mobile-Platform
#     # 2. If GPIO.23 is high, action = Pick
#     # 3. If GPIO.24 is high, action = Place

#     # 1. Pi Polls GPIO for event from Mobile-Platform
#     pick_event = GPIO.input(pick_pin)
#     place_event = GPIO.input(place_pin)

#     # 2. If GPIO.23 is high, action = Pick
#     if pick_event:
#         # mobile_platform_event = arm_comms_simulator(1, 0)
#         pass

#     # 3. If GPIO.24 is high, action = Place
#     elif place_event:
#         # mobile_platform_event = arm_comms_simulator(0, 1)
#         pass

#     time.sleep(5)
#     mobile_platform_event = 1

#     if mobile_platform_event:
#         print("Mobile Platform Go")
#         GPIO.output(go_pin, True)
#         place_event = 0
#         pick_event = 0

#         # 8. Mobile-Platform pulls GPIO.Pick or GPIO.Place low
#         # ? give mobile platform time to pull GPIO low
#         time.sleep(1)
#         pick_event = GPIO.input(pick_pin)

#         # 9. Pi pulls GPIO.Go low (for next cycle)
#         if not pick_event:
#             GPIO.output(go_pin, False)
#             mobile_platform_event = 0
#             print("Event Cycle Complete")
#             print("---------------------------------------\n")
