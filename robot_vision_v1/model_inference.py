from ultralytics import YOLO
import os

# Load a pretrained YOLO model (recommended for training)
model = YOLO('./model/w_best.pt')


def infer(img: str):

    result = model(img)
    # result = model.predict(img, conf=0.5, show=True, save=True, save_txt=True, save_conf=True)

    return result
