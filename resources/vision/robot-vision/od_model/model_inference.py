from ultralytics import YOLO
import os

# Load a pretrained YOLO model (recommended for training)
model = YOLO('w_best.pt')


for index, image in enumerate(os.listdir('./test_images')):
    print(image, index)

    # Perform object detection on an image using the model
    result = model(f"test_images/{image}")

    print(result[0].tojson())
