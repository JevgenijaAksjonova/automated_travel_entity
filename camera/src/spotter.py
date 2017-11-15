#! /usr/bin/env python
from __future__ import print_function

from camera.msg import PosAndImage

import rospy
import roslib
import numpy as np
from geometry_msgs.msg import Point as point_msg
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

from sensor_msgs.msg import CameraInfo
import numpy as np
import pprint
from image_geometry import PinholeCameraModel
pp = pprint.PrettyPrinter(indent = 4)
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String as String_msg

import sys
import rospkg
from os import path
rospack = rospkg.RosPack()
sys.path.insert(0,path.join(rospack.get_path("camera"),"src"))
from colorSegment import color_segment_image, color_2_rgb, colors
bridge = CvBridge()
#rec = Recognizer()

#Detects objects from the camera
class ObjectDetector:

    def __init__(self):
        
        #These values are set when we get new data and unset when we have used the data
        self._have_received_image = False
        self._have_received_depth = False
        self._has_received_cam_info = False
 
        self.obj_cand_pub = rospy.Publisher("/camera/object_candidates",PosAndImage,queue_size=10)
        
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.image_callback)
        
        #self.depth_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.depth_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect",Image,self.depth_callback)
        # This should probably be sub depth_registered/points. If we don't have it published,
        # check http://wiki.ros.org/rgbd_launch
        # and http://wiki.ros.org/realsense_camera#ROS_API
        # Registration => Matching two point sets
        # Rectification => Projecting two images onto a common plane, in our case to ease Registration
        # It is probably correct to use image_rect_color aswell.
        
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info",CameraInfo,self.info_callback)
        self.camera_model = PinholeCameraModel()          

        if DEBUGGING: 
            self.dbg_object_image = rospy.Publisher("camera/debug/object_candidate/image",Image,queue_size=1)
            self.dbg_img_pub = rospy.Publisher("/camera/debug/img",Image,queue_size=1)
   
    def image_callback(self,ros_image):
        try:
            self.rgb_image_msg = ros_image
            self._have_received_image = True
        except CvBridgeError as e:
            rospy.logdebug(e)
        #lab_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2LAB)
        #YCrCb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2YCrCb)
    
     # Get the depth image
    def depth_callback(self,depth_msg):
        #Only transforming the images we use might save computations depending on implementation of the bridge
        self.depth_msg = depth_msg 
        #bridge.imgmsg_to_cv2(ros_depth)
        self._have_received_depth = True 
    # Get the projection matrix
    def info_callback(self,info_message):
        self.camera_model.fromCameraInfo(info_message)
        self._has_received_cam_info = True
    
    #Process image :D
    def image_processing(self):

        if self._have_received_image and  self._has_received_cam_info and self._have_received_depth:
            
            rgb_image = bridge.imgmsg_to_cv2(self.rgb_image_msg,"passthrough")
            depth_image = bridge.imgmsg_to_cv2(self.depth_msg,"passthrough")
            
            ret_val = color_segment_image(rgb_image,depth_image,
                return_debug_image=DEBUGGING,
                apply_checks=False)
            
            if DEBUGGING:
                object_candidates, debug_img = ret_val
                debug_img = cv2.cvtColor(debug_img,cv2.COLOR_RGB2BGR)
                self.dbg_img_pub.publish(bridge.cv2_to_imgmsg(debug_img))
                if len(object_candidates) > 0:
                    self.dbg_object_image.publish(bridge.cv2_to_imgmsg(object_candidates[0].img,"rgb8"))
            else:
                object_candidates = ret_val
            print("object_candidates = {0}".format(object_candidates))
            for oc in object_candidates:
                print("sending oc = {0}".format(oc))
                oc_msg = self.get_oc_message(oc)
                self.obj_cand_pub.publish(oc_msg)
            
            
        self._have_received_image = False
        self._have_received_depth = False


    def get_oc_message(self,oc):
        point = np.array(self.camera_model.projectPixelTo3dRay(oc.center_point))
        point = point*oc.z

        obj_cand_msg = PosAndImage()
        obj_cand_msg.header.stamp = rospy.Time.now()
        obj_cand_msg.header.frame_id = "camera_link" #We might need to change this to it's propper value
        obj_cand_msg.pos.x = point[2]
        obj_cand_msg.pos.y = - point[0]
        obj_cand_msg.pos.z = - point[1]
        obj_cand_msg.image = bridge.cv2_to_imgmsg(oc.img)
        color_msg = String_msg()
        color_msg.data = oc.color
        obj_cand_msg.color = color_msg
        return obj_cand_msg
        

    #Detects objects untill shutdown. Permanently blocking.
    def detect_forever(self,rate=5):
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
