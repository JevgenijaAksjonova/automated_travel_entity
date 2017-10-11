#! /usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np

bridge = CvBridge()

#Set to true to get debug info

hue_threshold = 80

#Thresholds in hue for all collors
colors = ["green","red","blue","yellow"]

hue_thresholds = {
    "green" : (125,75),
    "red" : (35,0),
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


#Detects objects from the camera
class ObjectDetector:

    def __init__(self):
        
        #These values are set when we get new data and unset when we have used the data
        self._have_received_image = False
        self._have_recieved_depth = False
        
        self.obj_cand_pub = rospy.Publisher("/camera/object_candidates",PointStamped,queue_size=10)
        
        image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.image_callback)
        depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.depth_callback)
        info_sub = rospy.Subscriber("/camera/rgb/camera_info",CameraInfo,self.info_callback)
        
        if DEBUGGING: 
            self.dbg_img_pub = rospy.Publisher("/camera/debug/img",Image,queue_size=1)
            self.h_img_pub = rospy.Publisher("/camera/debug/hsv/h",Image,queue_size=1)

    def image_callback(self,ros_image):
        try:
            self.rgb_image = bridge.imgmsg_to_cv2(ros_image)
            self._have_received_image = True
        except CvBridgeError as e:
            rospy.logdebug(e)
        #lab_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2LAB)
        #YCrCb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2YCrCb)
    
    # Get the depth image
    def depth_callback(self,ros_depth):
        self.depth_image = bridge.imgmsg_to_cv2(ros_depth)
        self._have_received_depth = True 
    # Get the projection matrix
    def info_callback(self,info_message):
        self.P = info_message.P
    
    #Process image :D
    def image_processing(self):
        
        if self._have_received_image:
            for color in ["green"]: 
                hsv_image =  cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
                h_image = hsv_image[:,:,0]
                
                if DEBUGGING:
                    self.h_img_pub.publish(bridge.cv2_to_imgmsg(h_image,"mono8"))
    
                ret, thresh = cv2.threshold(h_image,hue_thresholds[color][1],hue_thresholds[color][0],cv2.THRESH_BINARY_INV)
                contours,_ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                #Plot different hue values, see what rgb they correspond ti
                #OR do it manually.
                for contour in contours:
    
                    bot_right = (int(contour[:,0,0].max()),int(contour[:,0,1].max()))
                    top_left = (int(contour[:,0,0].min()),int(contour[:,0,1].min()))
                    middle = (int(contour[:,0,0].mean()),int(contour[:,0,1].mean()))
    
                    #Mocking transform for now. OSKAR, yuor code goes here!
                    #Allso, we need to transform between depth frame and rgb frame here
    
                    obj_cand_msg = PointStamped()
                    obj_cand_msg.header.stamp = rospy.Time.now()
                    obj_cand_msg.header.frame_id = "/camera" #We might need to change this to it's propper value
                    obj_cand_msg.point.x = middle[0]
                    obj_cand_msg.point.y = middle[1]
                    obj_cand_msg.point.z = .1
                    self.obj_cand_pub.publish(obj_cand_msg)
                    
                    if DEBUGGING:    
                        cv2.drawContours(self.rgb_image,[contour],-1,color=color_2_rgb[color],thickness=-1)
                        cv2.circle(self.rgb_image,middle,radius=5,color=(0,0,0),thickness=2)
                        cv2.rectangle(self.rgb_image,top_left,bot_right,color=(0,0,0),thickness=2)
                    
                if DEBUGGING:
                    ros_out_image = bridge.cv2_to_imgmsg(self.rgb_image,"rgb8")
                    self.dbg_img_pub.publish(ros_out_image)

        self.have_received_image = False

    #Detects objects untill shutdown. Permanently blocking.
    def detect_forever(self,rate=10):
        rate = rospy.Rate(rate)
        rate.sleep()
        while not rospy.is_shutdown():
            self.image_processing()
            rate.sleep()         

def main():
    #Main fucntion. Put everything here
    global DEBUGGING
    DEBUGGING = rospy.get_param("/camera/debugging",True)
    
    rospy.init_node("object_candidate_spotter")
    
    spotter = ObjectDetector()
    spotter.detect_forever()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
