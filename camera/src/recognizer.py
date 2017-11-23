#! /usr/bin/env python
from __future__ import print_function

#ros imports
import rospy
from camera.srv import recognizer, recognizerResponse
#general imports
import numpy as np
import rospkg
from os import path, mkdir
rospack = rospkg.RosPack()

#Keras imports

try:
    from keras.preprocessing import image as keras_image
    from keras.models import load_model
    from keras.utils.generic_utils import CustomObjectScope
    from keras.applications.mobilenet import relu6, DepthwiseConv2D, preprocess_input
except ImportError:
    rospy.logerr("Failed to import neural net libs")

import cv2
from cv_bridge import CvBridge
bridge = CvBridge()

from time import ctime


class_indices = {
    'Orange Star': 6,
    'Red Hollow Cube': 9,
    'Purple Hollow Cross': 7,
    'Green Hollow Cube': 3,
    'Blue Cube': 0,
    'Red Sphere': 11,
    'Red Hollow Cylinder': 10,
    'Purple Star': 8,
    'Yellow Sphere': 13,
    'Blue Hollow Triangle': 1,
    'Orange Hollow Cross': 5,
    'Green Cube': 2,
    'Yellow Cube': 12,
    'Green Hollow Cylinder': 4
}
index_classes = {v: k for k, v in class_indices.items()}


class Recognizer:
    def __init__(self):
        base_dir = rospack.get_path("camera")
        self.model_path = path.join(base_dir, "data/models/model.spec")
        rospy.loginfo("self.model_path = " + self.model_path)

        rospy.loginfo("path.isfile(self.model_path) = " +
                      str(path.isfile(self.model_path)))
        with CustomObjectScope({
                'relu6': relu6,
                'DepthwiseConv2D': DepthwiseConv2D
        }):
            self.model = load_model(self.model_path)
            self.model._make_predict_function(
            )  #Quickfix for bug #2397 in keras

    def preprocess(self, image):
        input_shape = self.model.input_shape[1:3]
        image = keras_image.array_to_img(image).resize(input_shape)
        image = keras_image.img_to_array(image)
        image = preprocess_input(image)
        image = np.expand_dims(image, 0)

        return image

    def predict(self, image):
        image = self.preprocess(image)
        res = self.model.predict(image, steps=1)[0]
        probable_class = np.argmax(res)
        return probable_class, index_classes[probable_class], res[
            probable_class]

def save_classification(class_label,bgr_image):
    image = cv2.cvtColor(bgr_image,cv2.COLOR_BGR2RGB)
    if not path.isdir(class_label):
        mkdir(class_label)
    img_fn = path.join(class_label,"{time}.jpg".format(time=ctime()))
    cv2.imwrite(img_fn,image)

def main():
    rospy.init_node("recognizer_server")
    rec = Recognizer()

    def handle_recognize(request):
        req_image = request.image
        img = bridge.imgmsg_to_cv2(req_image)
        class_id, class_name, probability = rec.predict(img)
        save_classification(class_name,img)
        response = recognizerResponse()
        response.class_id.data = class_id
        response.class_name.data = class_name
        response.probability.data = probability
        return response

    rospy.Service("/camera/recognizer", recognizer, handle_recognize)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass