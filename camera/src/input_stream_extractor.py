#! /usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
bridge = CvBridge()
from time import ctime
from os import path,makedirs
save_folder = path.join("frames",ctime())
frame_counter = 0
def image_callback(image_msg):
    img = bridge.imgmsg_to_cv2(image_msg)
    fn = path.join(save_folder,"frame_{frame}.jpg")
    cv2.imwrite(fn,img)
def main():
    makedirs(save_folder)
    rospy.init_node("object_candidate_spotter")
    rospy.Subscriber("/camera/rgb/image_rect_color",Image, image_callback)
    rospy.spin()

if __name__ == "__main__":
    main()