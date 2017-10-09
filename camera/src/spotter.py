#! /usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()

hue_threshold = 80


thresholds = {
    "green" : (100,75)
}

def image_callback(ros_image):
    try:
        rgb_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        rospy.logdebug(e)
    #lab_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2LAB)
    #YCrCb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2YCrCb)
    hsv_image =  cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
#    rospy.loginfo("uhsv_image[:,:,0].max() = " + str(hsv_image[:,:,0].max()))
    
    h_image = hsv_image[:,:,0]
    max_thres = thresholds["green"][0]
    min_thres = thresholds["green"][1]
    rospy.loginfo("(max,min) = " + str((h_image.max(),h_image.min())))
    idx_max = (h_image > max_thres) 
    idx_min = (min_thres > h_image)
    h_image[idx_max] = 0
    h_image[idx_min] = 0
    rospy.loginfo("(max_thres,min_thres) = " + str((max_thres,min_thres)))
    ros_out_image = bridge.cv2_to_imgmsg(h_image,"mono8")
    dbg_img_pub.publish(ros_out_image)


def main():
    #Main fucntion. Put everything here
    pub = rospy.Publisher("/camera/object_candidates",PointStamped,queue_size=10)
    
    global dbg_img_pub
    dbg_img_pub = rospy.Publisher("/camera/debug/hsv",Image,queue_size=1)
    
    image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,image_callback)
    rospy.init_node("object_candidate_spotter")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        foo = PointStamped()
        foo.header.frame_id = "camera"
        foo.header.stamp = rospy.Time.now()
        foo.point.x = 1
        foo.point.y = 2
        foo.point.z = 3
#       rospy.loginfo("gelo world")
        pub.publish(foo)
        rate.sleep()
 
if __name__ == "__main__":
    try:
	    main()
    except rospy.ROSInterruptException:
        pass
