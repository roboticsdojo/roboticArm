from roboflow import Roboflow
import cv2
import numpy as np


# Initialize Inference Model
# rf = Roboflow(api_key="8kLxlVgusB4R1fsKc2DX")
# project = rf.workspace().project("roboken-object-detection")
# model = project.version(1).model


# Initialize Camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
# Set Dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)


def take_snapshot():
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            cv2.imwrite('camera_snapshot.jpg', frame)
            cv2.imshow("Raspberry Pi Camera V2", frame)
            cv2.waitKey(0)

        print("Failed to Capture")
        break

    cap.release()
    cv2.destroyAllWindows()


def video_stream():
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to Capture")
            break

        cv2.imshow("Raspberry Pi Camera V2", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


video_stream()

# # test-image
# test_image = 'test_image.jpg'

# # infer on a local image
# result = model.predict(test_image, confidence=40, overlap=30)

# # print result
# print(result.json())

# # save result
# result.save("test_result.jpg")
