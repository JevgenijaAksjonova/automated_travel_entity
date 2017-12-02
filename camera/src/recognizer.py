#! /usr/bin/env python
from __future__ import print_function
import rospy
import cv2
from cv_bridge import CvBridge
bridge = CvBridge()
import rospkg
from camera.srv import recognizer, recognizerResponse
rospack = rospkg.RosPack()
from os import path, mkdir
from time import ctime
import sys
camera_base = rospack.get_path("camera")
sys.path.insert(0,path.join(camera_base,"src"))
from recognizer_model import Recognizer

def save_classification(class_label, bgr_image):
    image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    if not path.isdir(class_label):
        mkdir(class_label)
    img_fn = path.join(class_label, "{time}.jpg".format(time=ctime()))
    cv2.imwrite(img_fn, image)


def main():
    rospy.init_node("recognizer_server")
    model_path = path.join(camera_base, "data/models/No color based classes")
    rec = Recognizer(model_path)


    def handle_recognize(request):
        req_image = request.image
        img = bridge.imgmsg_to_cv2(req_image)
        class_id, class_name, probability = rec.predict(img)
        save_classification(class_name, img)
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
