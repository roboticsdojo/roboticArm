from roboflow import Roboflow

rf = Roboflow(api_key="8kLxlVgusB4R1fsKc2DX")
project = rf.workspace().project("roboken-object-detection")
model = project.version(1).model


# test-image
test_image = 'test_image.jpg'

# infer on a local image
result = model.predict(test_image, confidence=40, overlap=30)

# print result
print(result.json())

# save result
result.save("test_result.jpg")
