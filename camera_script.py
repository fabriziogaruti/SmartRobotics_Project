import cv2
import torch

def camera_script(yooloModel):
    # Opens the inbuilt camera of laptop to capture video.
    cap = cv2.VideoCapture(0)
    i = 0

    while (cap.isOpened()):
        ret, frame = cap.read()
        if(i == 10):
            results = yooloModel(frame)
            #TODO: gestire se non c'è una persona
            df = results.pandas()
            person = df.xyxy[0][df.xyxy[0]['class'] == 0]
            cv2.rectangle(frame, (person['xmin'], person['ymax']), (person['xmax'], person['ymin']), (0, 255, 0), 3)
            i=0
        # This condition prevents from infinite looping
        # incase video ends.
        if ret == False:
            break

        # Save Frame by Frame into disk using imwrite method
        #cv2.imwrite('Images/'+'Frame' + str(i) + '.jpg', frame)
        cv2.imshow('Frame', frame)
        i += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    model = torch.hub.load('ultralytics/yolov3', 'yolov3')
    camera_script(model)