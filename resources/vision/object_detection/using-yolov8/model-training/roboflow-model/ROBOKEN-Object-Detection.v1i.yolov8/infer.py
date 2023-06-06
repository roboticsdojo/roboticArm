from roboflow import Roboflow

rf = Roboflow(api_key="8kLxlVgusB4R1fsKc2DX")
project = rf.workspace().project("roboken-object-detection")
model = project.version(1).model


# test-images
trailer = "./valid/images/20230525_163046_jpg.rf.afb9e90b58c1d28fba6479be64598bd9.jpg"
wheel = "./valid/images/20230525_163804_jpg.rf.3f7448b1fc2fd0a4fabf211878036847.jpg"
engine = "./valid/images/20230525_165103_jpg.rf.196e628a1fdcaf12f1fb5490a450da4c.jpg"


# infer on a local image
print(model.predict(engine, confidence=40, overlap=30).json())

# visualize your prediction
model.predict(engine, confidence=40, overlap=30).save("engine.jpg")

# infer on an image hosted elsewhere
# print(model.predict("URL_OF_YOUR_IMAGE", hosted=True, confidence=40, overlap=30).json())
