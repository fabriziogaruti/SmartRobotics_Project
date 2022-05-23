import torch

# Model
model = torch.hub.load('ultralytics/yolov3', 'yolov3')  # or yolov3-spp, yolov3-tiny, custom

# Images
img = 'Data/base.jpeg'  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img)

# Results
#results.print()
results.show()
#results.save()
#print(results)
df = results.pandas()
print(df.xyxy[0][df.xyxy[0]['class']==0])