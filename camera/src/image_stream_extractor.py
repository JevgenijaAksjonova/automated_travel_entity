#! /usr/bin/env python
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
bridge = CvBridge()
from time import ctime
from os import path, makedirs

save_folder = path.join("frames", ctime())

frame_count = 0
SAVE_EVERY = 10
def image_callback(image_msg):
    global frame_count
    if frame_count % SAVE_EVERY == 0:
        img = bridge.imgmsg_to_cv2(image_msg)
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        fn = path.join(
            save_folder, "frame_{0:06d}.jpg".format(frame_count))
        cv2.imwrite(fn, img)
        rospy.loginfo("Wrote {file}".format(file=fn))
    frame_count = frame_count + 1


def main():
    makedirs(save_folder)
    rospy.init_node("image_stream_extractor")
    rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_callback,queue_size=10000)
    rospy.spin()


if __name__ == "__main__":
    main()
