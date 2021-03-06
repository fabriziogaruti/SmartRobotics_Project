import cv2
import torch
import time
from AI_Detection.detect_faces import get_face_bounding_box, get_face_bounding_box_pytorch
# import mtcnn
from facenet_pytorch import MTCNN
import paho.mqtt.client as mqtt
from datetime import datetime
from publisher_mqtt import on_connect, on_message
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

stato=0
resize_factor = 0.7


def classify_position(bbox_body, bbox_face, frame):
    global stato
    filename = "pub_vel_ws/file.txt"

    #Case 1: hands on the top
    face_w = abs(bbox_face["xmax"] - bbox_face["xmin"])
    face_h = abs(bbox_face["ymax"] - bbox_face["ymin"])
    body_w = abs(bbox_body["xmax"] - bbox_body["xmin"])
    body_h = abs(bbox_body["ymax"] - bbox_body["ymin"])
    center = [int((bbox_face["xmin"] + bbox_face["xmax"]) / 2), int((bbox_face["ymin"] + bbox_face["ymax"]) / 2)]
    body_delta_left = abs(center[0] - bbox_body["xmin"])
    body_delta_right = abs(center[0] - bbox_body["xmax"])
    body_delta_up = abs(center[1] - bbox_body["ymin"])

    #Hands High
    if bbox_body["ymin"] < bbox_face["ymin"] - face_h:
        # print("Hands Up")
        cv2.rectangle(frame, (10, 2), (100, 20), (255, 255, 255), -1)
        cv2.putText(frame, "Hands Up", (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        if stato != 1:
            with open(filename, "w") as f:
                f.write("1")
            client.publish(TOPIC, 1)
            stato = 1

    #Hands Left
    elif bbox_body["xmin"] < bbox_face["xmin"] - 3 * face_w:
        # print("Hands Left")
        cv2.rectangle(frame, (10, 2), (100, 20), (255, 255, 255), -1)
        cv2.putText(frame, "Hands Left", (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        if stato != 2:
            with open(filename, "w") as f:
                f.write("2")
            client.publish(TOPIC, 2)
            stato = 2

    # Hands Right
    elif bbox_body["xmax"] > bbox_face["xmax"] + 3 * face_w:
        # print("Hands Right")
        cv2.rectangle(frame, (10, 2), (100, 20), (255, 255, 255), -1)
        cv2.putText(frame, "Hands Right", (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        if stato != 3:
            with open(filename, "w") as f:
                f.write("3")
            client.publish(TOPIC, 3)
            stato = 3

    else:
        # print("Hands Normal")
        cv2.rectangle(frame, (10, 2), (100, 20), (255, 255, 255), -1)
        cv2.putText(frame, "Hands Normal", (15, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
        if stato != 0:
            with open(filename, "w") as f:
                f.write("0")
            stato = 0
            client.publish(TOPIC, stato)


def camera_script(yooloModel, face_detector):
    # Opens the inbuilt camera of laptop to capture video.
    cap = cv2.VideoCapture(0)
    i = 0

    while cap.isOpened():
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (int(frame.shape[1] * resize_factor), int(frame.shape[0] * resize_factor)))
        # time.sleep(0.1)
        results = yooloModel(frame)
        df = results.pandas()
        person = df.xyxy[0][df.xyxy[0]['class'] == 0]
        if not person.empty:
            person = person.iloc[0]
            # print(person['xmin'])
            cv2.rectangle(frame, (int(person['xmin']), int(person['ymin'])), (int(person['xmax']), int(person['ymax'])), (0, 255, 0), 3)

            # bbox = get_face_bounding_box(face_detector, frame)
            bbox = get_face_bounding_box_pytorch(face_detector, frame)
            if bbox is not None:
                cv2.rectangle(frame, (int(bbox['xmin']), int(bbox['ymin'])), (int(bbox['xmax']), int(bbox['ymax'])),
                              (255, 0, 0), 3)
                classify_position(person, bbox, frame)
            # grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # cv2.imwrite('Images/' + 'Frame' + str(i) + '.jpg', frame)

        frame = cv2.resize(frame, (int(frame.shape[1] / resize_factor), int(frame.shape[0] / resize_factor)))
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
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s').to(device)
    # face_detector = mtcnn.MTCNN()
    # define our extractor
    face_detector = MTCNN(margin=14, factor=0.6, keep_all=True,)

    BROKER = 'test.mosquitto.org'
    TOPIC = 'bot/fsm-state'
    client = mqtt.Client(clean_session=True)

    # events --> callback association
    client.on_connect = on_connect
    client.on_message = on_message

    # client --> broker connection

    client.connect(BROKER)
    client.loop_start()
    camera_script(model, face_detector)
