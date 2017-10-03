#! /usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import PointStamped


def main():
    #Main fucntion. Put everything here
    pub = rospy.Publisher("/camera/object_candidates",PointStamped,queue_size=10)
    rospy.init_node("talker")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        foo = PointStamped()
        foo.header.frame_id = "camera"
        foo.header.stamp = rospy.Time.now()
        foo.point.x = 1
        foo.point.y = 2
        foo.point.z = 3
        rospy.loginfo("gelo world")
        pub.publish(foo)
        rate.sleep()
        
if __name__ == "__main__":
    try:
	    main()
    except rospy.ROSInterruptException:
        pass
