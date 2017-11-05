#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
from camera.srv import recognizer
from camera.msg import PosAndImage
from sensor_msgs.msg import Image

from ras_msgs.msg import RAS_Evidence

class Mother:
    has_received_obj_cand = False
    def __init__(self):
        rospy.Subscriber("/camera/object_candidates",PosAndImage,
            callback=self.obj_cand_callback)
        self.evidence_pub = rospy.Publisher("evidence_publisher",RAS_Evidence,queue_size=1)
        self.rec_srv = rospy.ServiceProxy("/camera/recognizer",recognizer,persistent=True)
    
    def obj_cand_callback(self,obj_cand_msg):
        self.obj_cand_msg = obj_cand_msg
        self.has_received_obj_cand = True


    def mother_forever(self,rate=.2):
        rate = rospy.Rate(rate)
        rate.sleep()
        while not rospy.is_shutdown():
            print("loop")
            if self.has_received_obj_cand:
                print("encoding =",self.obj_cand_msg.image.encoding)
                try:
                    resp = self.rec_srv(self.obj_cand_msg.image)

                    class_name = resp.class_name
                    class_id = resp.class_id
                    confidence = resp.probability

                    evidence_msg = RAS_Evidence()
                    evidence_msg.group_number = 3
                    evidence_msg.image_evidence = self.obj_cand_msg.image
                    evidence_msg.object_id = class_name if confidence > .9 else "an_object"
                    print("Sending evidence")                    
                    self.evidence_pub.publish(evidence_msg)
                except rospy.ServiceException, e:
                    rospy.loginfo("Service call failed: " + str(e))
            self.has_received_obj_cand = False
            rate.sleep()

def main():        
    rospy.init_node("recognizer_server")    
    rospy.wait_for_service("/camera/recognizer")
    m = Mother()
    m.mother_forever()






if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass