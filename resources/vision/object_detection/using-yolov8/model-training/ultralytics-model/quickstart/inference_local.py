from ultralytics import YOLO

# Create a new YOLO model from scratch
# model = YOLO('yolov8n.yaml')

# Load a pretrained YOLO model (recommended for training)
model = YOLO('yolov8n.pt')

# Train the model using the 'coco128.yaml' dataset for 3 epochs
# results = model.train(data='coco128.yaml', epochs=3)

# Train YOLOv8n on the COCO128 dataset for 100 epochs at image size 640
# model.train(data='coco128.yaml', epochs=100, imgsz=640)

# Evaluate the model's performance on the validation set
# results = model.val()

# Perform object detection on an image using the model
results = model('./test-images/bus.jpg')

# Export the model to ONNX format
# success = model.export(format='onnx')
