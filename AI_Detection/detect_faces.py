import mtcnn
import matplotlib.pyplot as plt
# import tensorflow as tf; print(tf.config.list_physical_devices('GPU'))


# draw an image with detected objects
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


if __name__ == "__main__":
    # load image from file
    filename = "../Data/base.jpeg"
    pixels = plt.imread(filename)
    print("Shape of image/array:", pixels.shape)
    imgplot = plt.imshow(pixels)
    plt.show()

    detector = mtcnn.MTCNN()
    # detect faces in the image
    faces = detector.detect_faces(pixels)
    for face in faces:
        print(face)

    # display faces on the original image
    draw_facebox(filename, faces)
