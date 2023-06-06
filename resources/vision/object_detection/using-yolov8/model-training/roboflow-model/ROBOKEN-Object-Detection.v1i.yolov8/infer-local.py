from roboflow import Roboflow

rf = Roboflow(api_key="8kLxlVgusB4R1fsKc2DX")
project = rf.workspace().project("roboken-object-detection")
model = project.version(1, local="http://localhost:9001/").model


# test-images
trailer = "./valid/images/20230525_163046_jpg.rf.afb9e90b58c1d28fba6479be64598bd9.jpg"
wheel = "./valid/images/20230525_163804_jpg.rf.3f7448b1fc2fd0a4fabf211878036847.jpg"
engine = "./valid/images/20230525_165103_jpg.rf.196e628a1fdcaf12f1fb5490a450da4c.jpg"


# infer on a local image
prediction = model.predict(wheel, confidence=40, overlap=30)

# print results
print(prediction.json())

# visualize your prediction | Save image
prediction.save("./wheel.jpg")
