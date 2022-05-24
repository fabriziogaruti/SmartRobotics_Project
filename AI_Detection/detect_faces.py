import mtcnn
import matplotlib.pyplot as plt
# import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))


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
    x, y, width, height = faces[0]['box']
    # print(x, y, width, height)
    '''xmin = int(x - width / 2)
    xmax = int(x + width / 2)
    ymin = int(y - height / 2)
    ymax = int(y + height / 2)
    '''
    xmin = int(x)
    xmax = int(x + width)
    ymin = int(y +height)
    ymax = int(y)
    print("Vertici:", xmin, xmax, ymin, ymax)
    return [xmin, xmax, ymin, ymax]


if __name__ == "__main__":
    # load image from file
    filename = "../Data/base.jpeg"
    img = plt.imread(filename)
    print("Shape of image/array:", img.shape)
    imgplot = plt.imshow(img)
    plt.show()

    # Detector e get della bbox (xmin xmax ymin ymax)
    detector = mtcnn.MTCNN()
    bbox = get_face_bounding_box(detector, img)

    # NON UTILIZZARE SOTTO
    # detect faces in the image
    faces = detector.detect_faces(img)
    for face in faces:
        print(face)
    # display faces on the original image
    draw_facebox(filename, faces)
