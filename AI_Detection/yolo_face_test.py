import torch
import mtcnn
from detect_faces import get_face_bounding_box
import matplotlib.pyplot as plt

# Model
model = torch.hub.load('ultralytics/yolov3', 'yolov3')  # or yolov3-spp, yolov3-tiny, custom
detector = mtcnn.MTCNN()

# Images
img_name = '../Data/base.jpeg'  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img_name)

# Results
#results.print()
#results.save()
#print(results)

# results.show()
df = results.pandas()
# print(df.xyxy[0][df.xyxy[0]['class']==0])

xyxy = df.xyxy[0][df.xyxy[0]['class']==0]
'''print(xyxy.iloc[0].tolist())
xmax = xyxy.iloc[0, 0]
ymin = xyxy.iloc[0, 1]
print(xmax)
print(ymin)'''
xyxy = xyxy.iloc[0].tolist()
print(xyxy)
print(tuple(xyxy))

img = plt.imread(img_name)
bbox = get_face_bounding_box(detector, img)
center = [int((bbox[0] + bbox[1])/2), int((bbox[2] + bbox[3])/2)]
print("Center:", center)


