# import mtcnn
from facenet_pytorch import MTCNN
import matplotlib.pyplot as plt
# import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))
import time
import cv2
import torchvision.transforms as transforms
import torch
import warnings
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# draw an image with detected objects with MatplotLib
def draw_facebox(filename, result_list):
    # load the image
    data = plt.imread(filename)
    # plot the image
    plt.imshow(data)
    # get the context for drawing boxes
    ax = plt.gca()
    # plot each box
    for result in result_list:
        # get coordinates
        x, y, width, height = result['box']
        # create the shape
        rect = plt.Rectangle((x, y), width, height, fill=False, color='orange')
        # draw the box
        ax.add_patch(rect)
    # show the plot
    plt.show()


def get_face_bounding_box(detector, img):
    faces = detector.detect_faces(img)
    if len(faces) != 0:
        x, y, width, height = faces[0]['box']
        # print(x, y, width, height)
        xmin = x
        xmax = x + width
        ymin = y
        ymax = y + height
        print("Vertici:", xmin, xmax, ymin, ymax)
        print("Vertici:", x, y, width, height)
        return {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax}
    return None


def get_face_bounding_box_pytorch(detector, img):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        faces = detector.detect(img)

    if faces[1][0] is not None and len(faces[0]) != 0:
        xmin, ymin, xmax, ymax = faces[0][0]
        # print("Vertici:", xmin, xmax, ymin, ymax)
        # print("Vertici:", x, y, width, height)
        return {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax}
    return None


if __name__ == "__main__":
    # load image from file
    filename = "../Data/base.jpeg"
    img = plt.imread(filename)
    print("Shape of image/array:", img.shape)
    # imgplot = plt.imshow(img)
    # plt.show()

    # Detector e get della bbox (xmin ymin xmax ymax)
    # detector = mtcnn.MTCNN()

    face_detector = MTCNN(margin=14, factor=0.6, keep_all=True,)
    # Prima possibility
    bbox = get_face_bounding_box_pytorch(face_detector, img)

    '''# Second possibility
    # detect faces in the image
    t1 = time.time()
    faces = detector.detect_faces(img)
    t2 = time.time()
    print("Tempo impiegato: {:.3f}".format(t2-t1))

    for face in faces:
        print(face)
    # display faces on the original image
    draw_facebox(filename, faces)'''
