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
center = [int((xyxy_face["xmin"] + xyxy_face["xmax"])/2), int((xyxy_face["ymin"] + xyxy_face["ymax"])/2)]
print("\nFace center:", center)
print("Bbox body:", xyxy)
width_testa = abs(xyxy_face["xmin"] - xyxy_face["xmax"])

print("\nDelta x left:  {:.0f}".format(abs(center[0] - xyxy[0])))
print("Delta x right:   {:.0f}".format(abs(center[0] - xyxy[2])))
print("Delta y up:      {:.0f}".format(abs(center[1] - xyxy[1])))

print("\nDelta x left:  {:.3f}".format(abs(center[0] - xyxy[0]) / xyxy_face["xmin"]))
print("Delta x right:   {:.3f}".format(abs(center[0] - xyxy[2]) / xyxy_face["xmax"]))
print("Delta y up:      {:.3f}".format(abs(center[1] - xyxy[1]) / xyxy_face["ymin"]))

# PIXELS left, right, up
# Base : 239 237 132
# Left : 844 320 125
# Right : 285 783 132
# Up : 291 331 499

# PIXELS / WIDTH, HEIGHT
# Base : 0.383 0.324 0.310
# Left : 1.024 0.342 0.383
# Right : 0.896 1.830 0.348
# Up : 0.425 0.414 0.884
