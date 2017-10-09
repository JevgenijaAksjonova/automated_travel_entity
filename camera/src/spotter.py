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

#Thresholds in hue for all collors
colors = ["green","red","blue","yellow"]

hue_thresholds = {
    "green" : (125,75),
    "red" : (255,0),
    "blue" : (0,0),
    "yellow" : (0,0) ,
}
color_2_rgb = {
    "green" : (0,255,0),
    "red" : (255,0,0),
    "blue" : (0,0,255),
    "yellow" : (0,255,255),
}
#Todo: add thresholds for value aswell, for increased robustness

def image_callback(ros_image):
    try:
        rgb_image = bridge.imgmsg_to_cv2(ros_image)
    except CvBridgeError as e:
        rospy.logdebug(e)
    #lab_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2LAB)
    #YCrCb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2YCrCb)
    for color in colors: 
        hsv_image =  cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        h_image = hsv_image[:,:,0]
        ret, thresh = cv2.threshold(h_image,hue_thresholds[color][1],hue_thresholds[color][0],cv2.THRESH_BINARY_INV)
        contours,_ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            cv2.drawContours(rgb_image,[contour],-1,color=color_2_rgb[color],thickness=-1)
            middle = (int(contour[:,0,0].mean()),int(contour[:,0,1].mean()))
            cv2.circle(rgb_image,middle,radius=5,color=(0,0,0),thickness=2)

            bot_right = (int(contour[:,0,0].max()),int(contour[:,0,1].max()))
            top_left = (int(contour[:,0,0].min()),int(contour[:,0,1].min()))
            cv2.rectangle(rgb_image,top_left,bot_right,color=(0,0,0),thickness=2)
        #Find the center of the
        ros_out_image = bridge.cv2_to_imgmsg(rgb_image,"rgb8")
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
