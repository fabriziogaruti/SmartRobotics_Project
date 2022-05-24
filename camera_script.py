import cv2
import torch
import time
from AI_Detection.detect_faces import get_face_bounding_box
import mtcnn

def classify_position(bbox_body, bbox_face, frame):
    #Case 1: hands on the top
    face_w = bbox_face[1] - bbox_face[0]
    face_h = bbox_face[2] - bbox_face[3]
    body_w = bbox_body[1] - bbox_body[0]
    body_h = bbox_body[2] - bbox_body[3]
    print("face_h", face_h)
    print("bbox_body_ymax",bbox_body[3])
    print("bbox_face_ymax", bbox_face[3])
    if bbox_body[3] < bbox_face[3] - face_h:
        print("mani alzate")
        cv2.rectangle(frame, (10, 2), (100, 20), (255, 255, 255), -1)
        cv2.putText(frame, "mani alzate", (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

def camera_script(yooloModel, face_detector):
    # Opens the inbuilt camera of laptop to capture video.
    cap = cv2.VideoCapture(1)
    i = 0

    while (cap.isOpened()):
        ret, frame = cap.read()
        # print("prova")
        time.sleep(0.1)
        results = yooloModel(frame)
        df = results.pandas()
        person = df.xyxy[0][df.xyxy[0]['class'] == 0]
        if not person.empty:
            person = person.iloc[0]
            # print(person['xmin'])
            cv2.rectangle(frame, (int(person['xmin']), int(person['ymin'])), (int(person['xmax']), int(person['ymax'])), (0, 255, 0), 3)
            bbox = get_face_bounding_box(face_detector, frame)
            cv2.rectangle(frame, (int(bbox['xmin']), int(bbox['ymin'])), (int(bbox['xmax']), int(bbox['ymax'])),
                          (255, 0, 0), 3)
            classify_position([person['xmin'],person['xmax'],person['ymin'],person['ymax']], bbox, frame)
            #grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)
            #cv2.imwrite('Images/' + 'Frame' + str(i) + '.jpg', frame)
        else:
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)
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
    face_detector = mtcnn.MTCNN()
    camera_script(model, face_detector)