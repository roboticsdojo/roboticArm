import cv2
import time
import random

video = cv2.VideoCapture(0)

count = 0
frame_count = 0
random.seed(time.time())

print("Press SPACE Key to start Video-to-Image Capture")

while True:
    ret, frame = video.read()

    if not ret:
        print("Failed to get frame")
        break

    fps = video.get(cv2.CAP_PROP_FPS)
    cv2.imshow(f"Frame: {fps} fps", frame)

    struct_time_tuple = time.localtime()  # get struct_time
    time_string = time.strftime("%H:%M:%S", struct_time_tuple)

    count += 1

    key = cv2.waitKey(1)
    # SPACE pressed
    if key % 256 == 32:
        # At 30 fps, get 10 frames every second by only storing frames divisible by 3
        if count % 3 == 0:
            cv2.imwrite(
                f'./images/blue_wheel/d_table/image_{frame_count}_{random.randrange(200)}.jpg', frame)
            print(f"t :> {time_string} | {count} | {frame_count}")
            frame_count += 1

    if frame_count == 100:
        print(f"Video-to-Image Capture complete | Frames: {frame_count}")
        break

    if key == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
