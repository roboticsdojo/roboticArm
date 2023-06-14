from roboflow import Roboflow
import cv2
import numpy as np
import json


# Initialize Inference Model
rf = Roboflow(api_key="8kLxlVgusB4R1fsKc2DX")
project = rf.workspace().project("roboken-object-detection")
model = project.version(1).model


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


def infer_from_snapshot():
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            # cv2.imwrite('infer_from_snapshot.jpg', frame)
            # cv2.imshow("Raspberry Pi Camera V2", frame)
            # cv2.waitKey(0)

            # Infer on snapshot
            result = model.predict(frame, confidence=40, overlap=30)

            # Print result
            print(result.json())

        print("Failed to Capture")
        break

    cap.release()
    cv2.destroyAllWindows()


def infer_from_video_stream():
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to Capture")
            break

        # Show video stream
        # cv2.imshow("Raspberry Pi Camera V2", frame)

        # Infer on video stream (frame by frame)
        result = model.predict(frame, confidence=40, overlap=30)

        print(result.json())
        # print(f"Prediction_Result: {result.json()['predictions']}")

        #! Save result as image (visualize) [FAILED]
        # ? Convert to image object
        # if result.json()['predictions']:
        #     result.save("video_inference_result.jpg")
        #     print("Save result as image [SUCCESS]")

        #! Save result to JSON file [FAILED]
        # with open('result.json', 'a') as f:
        #     json.dump(result.json(), f)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# infer_from_video_stream()
infer_from_snapshot()
# video_stream()

# # test-image
# test_image = 'test_image.jpg'

# # infer on a local image
# result = model.predict(test_image, confidence=40, overlap=30)

# # print result
# print(result.json())

# # save result
# result.save("test_result.jpg")
