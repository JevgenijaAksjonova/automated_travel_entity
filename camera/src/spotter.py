#! /usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


bridge = CvBridge()

def image_callback(image):
    try:
        cv_image = bridge.imgmsg_to_cv2(image)
    except CvBridgeError as e:
        rospy.logdebug(e)
    
def main():
    #Main fucntion. Put everything here
    pub = rospy.Publisher("/camera/object_candidates",PointStamped,queue_size=10)
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
#        rospy.loginfo("gelo world")
        pub.publish(foo)
        rate.sleep()
 
if __name__ == "__main__":
    try:
	    main()
    except rospy.ROSInterruptException:
        pass
