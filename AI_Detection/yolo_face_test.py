import torch
import mtcnn
from detect_faces import get_face_bounding_box
import matplotlib.pyplot as plt

# Model
model = torch.hub.load('ultralytics/yolov3', 'yolov3')  # or yolov3-spp, yolov3-tiny, custom
detector = mtcnn.MTCNN()

# Images
img_name = '../Data/base.jpeg'  # or file, Path, PIL, OpenCV, numpy, list
# img_name = '../Data/left.jpeg'
# img_name = '../Data/right.jpeg'
# img_name = '../Data/up.jpeg'

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
# print(xyxy)
# print(tuple(xyxy))

img = plt.imread(img_name)
xyxy_face = get_face_bounding_box(detector, img)
center = [int((xyxy_face[0] + xyxy_face[2])/2), int((xyxy_face[1] + xyxy_face[3])/2)]
print("\nFace center:", center)
print("Bbox body:", xyxy)
width_testa = abs(xyxy_face[0] - xyxy_face[2])

print("\nDelta x left:  {:.0f}".format(abs(center[0] - xyxy[0])))
print("Delta x right:   {:.0f}".format(abs(center[0] - xyxy[2])))
print("Delta y up:      {:.0f}".format(abs(center[1] - xyxy[1])))

print("\nDelta x left:  {:.3f}".format(abs(center[0] - xyxy[0])/(xyxy_face[0] - width_testa)))
print("Delta x right:   {:.3f}".format(abs(center[0] - xyxy[2])/(xyxy_face[2] + width_testa)))
print("Delta y up:      {:.3f}".format(abs(center[1] - xyxy[1])/xyxy_face[1]))

# PIXELS
# Base : 239 237 132
# Up : 291 331
# Right : 285 783
# Left : 844 320

# PIXELS / WIDTH, HEIGHT
# Base : 0.206 0.204 0.241
