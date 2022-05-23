import cv2
import torch

def camera_script(yooloModel):
    # Opens the inbuilt camera of laptop to capture video.
    cap = cv2.VideoCapture(0)
    i = 0

    while (cap.isOpened()):
        ret, frame = cap.read()
        print("prova")
        if(i == 30):
            results = yooloModel(frame)
            df = results.pandas()
            person = df.xyxy[0][df.xyxy[0]['class'] == 0]
            if not person.empty:
                print(person['xmin'])
                cv2.rectangle(frame, (int(person['xmin']), int(person['ymax'])), (int(person['xmax']), int(person['ymin'])), (0, 255, 0), 3)
                #grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                cv2.imshow('Frame', frame)
                cv2.waitKey(1000)
                #cv2.imwrite('Images/' + 'Frame' + str(i) + '.jpg', frame)
                i = 0
        # This condition prevents from infinite looping
        # incase video ends.
        if ret == False:
            break

        # Save Frame by Frame into disk using imwrite method
        #cv2.imwrite('Images/'+'Frame' + str(i) + '.jpg', frame)
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        i += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
    camera_script(model)