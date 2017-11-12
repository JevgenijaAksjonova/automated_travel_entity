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

bridge = CvBridge()
#rec = Recognizer()

#Thresholds in hue for all collors
colors = ["green","red","blue_low","blue_high","yellow"]


#map of (lower,upper) thresholds in hsv for the respective colors
#HSV values in open cv are given in ranges (0-180,0-255,0-255) respectively
color_2_rgb = {
    "green" : (0,255,0),
    "red" : (255,0,0),
    "blue_low" : (0,0,255),
    "blue" : (0,0,255),
    "yellow" : (0,255,255),
}

# Takes a standard hsv point and transforms it into the form used by opencv.
def hsv_2_opencv(hsv):
    h = hsv[0]; s=hsv[1]; v=hsv[2]
#    h = max(h,0); h = min(h,360) #in range [0,360]
#    s = max(s,0); s = min(s,100)
#    v = max(v,0); v = min(v,100)
    return np.array([int(h//2),int(s*2.55),int(v*2.55)],dtype=np.uint8)
color_space = np.zeros((255,180,3),dtype=np.uint8)
for x in range(255):
    for y in range(180):
        color_space[x,y,1] = x
        color_space[x,y,0] = y
        color_space[x,y,2] = 255
cv_color_space =  cv2.cvtColor(color_space, cv2.COLOR_HSV2BGR)
cv2.imwrite('very_secret_open_cv_hsv_space.png',cv_color_space)


def extract_object_image((x_mid,y_mid),(x_min,y_min),(x_max,y_max),image):
    #Calculate the width of the window
    window_width = max(x_max - x_min,y_max - y_min)
    alignment = int((window_width//2) * 1.5)

    #Create preliminary new window
    new_x_min = x_mid - alignment
    new_x_max = x_mid + alignment
    new_y_min = y_mid - alignment
    new_y_max = y_mid + alignment

    #Check if resulting window is outside of image
    if new_x_max >= image.shape[0]:
        new_x_mid = x_mid - (new_x_max - image.shape[0])
    elif new_x_min < 0:
        new_x_mid = x_mid - new_x_min
    else:
        new_x_mid = x_mid

    if new_y_max >= image.shape[0]:
        new_y_mid = y_mid - (new_y_max - image.shape[0])
    elif new_y_min < 0:
        new_y_mid = y_mid - new_y_min
    else:
        new_y_mid = y_mid
    
    #Create new window
    new_x_min = new_x_mid - alignment
    new_x_max = new_x_mid + alignment
    new_y_min = new_y_mid - alignment
    new_y_max = new_y_mid + alignment

    return image[new_y_min:new_y_max,new_x_min:new_x_max,:]
    
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
        self.load_hsv_thresholds()
        self.camera_model = PinholeCameraModel()          

        if DEBUGGING: 
            self.dbg_object_pos = rospy.Publisher("camera/debug/object_candidate/pos",point_msg,queue_size=1)
            self.dbg_object_image = rospy.Publisher("camera/debug/object_candidate/image",Image,queue_size=1)
            self.dbg_img_pub = rospy.Publisher("/camera/debug/img",Image,queue_size=1)
            self.h_img_pub = rospy.Publisher("/camera/debug/hsv/h",Image,queue_size=1)
            self.h_mask_pub = rospy.Publisher("/camera/debug/h/mask",Image,queue_size=1)
            self.hsv_scale_pub = rospy.Publisher("/camera/debug/hsv/scale/",Image,queue_size=1)


    def load_hsv_thresholds(self):
        self.set_hsv_thresholds()
        #thresholds = rospy.get_param("/camera/hsv_thresholds")
        #self.hsv_thresholds = dict()
        #for color,thresh in thresholds.iteritems():
        #    assert(color in colors)
        #    lower = hsv_2_opencv(thresholds[color]["lower"])
        #    upper = hsv_2_opencv(thresholds[color]["upper"])
        #    self.hsv_thresholds[color] = (lower,upper)
   
    def set_hsv_thresholds(self):
        self.hsv_thresholds = {
            "red":(np.array([110,240,10]),np.array([120,255,255])),
            "green":(np.array([40,130,50]),np.array([85,255,200])),
            "yellow":(np.array([0,180,100]),np.array([0,255,255])),
            "blue":(np.array([18,100,15]),np.array([35,255,235])),
            "blue_high":(np.array([150,0,0]),np.array([180,255,255]))}

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
    
    _mask_kernel = np.ones((3,3),np.uint8)
    def compute_mask(self,image,(lower,upper)):

        mask = cv2.inRange(image,lower,upper)

        cv2.morphologyEx(mask,cv2.MORPH_OPEN,
            kernel = self._mask_kernel,
            dst=mask,
            iterations = 10)

        return mask

    #Process image :D
    def image_processing(self):
        
        if DEBUGGING: 
            self.hsv_scale_pub.publish(bridge.cv2_to_imgmsg(cv_color_space,"rgb8"))
        if self._have_received_image and  self._has_received_cam_info and self._have_received_depth:
            rgb_image = bridge.imgmsg_to_cv2(self.rgb_image_msg,"rgb8")
            depth_image = bridge.imgmsg_to_cv2(self.depth_msg,"passthrough")
            hsv_image =  cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
            #res = rec.predict(rgb_image)
            #cv2.GaussianBlur(hsv_image,(11,11),3,hsv_image)
            if DEBUGGING:
                mask_union = None
                rgb_dbg = rgb_image.copy()
                self.load_hsv_thresholds()
            
            for color in ["blue","green"]:
                mask = self.compute_mask(hsv_image,self.hsv_thresholds[color])
                
                if DEBUGGING:
                    if mask_union is None:
                        mask_union = np.zeros_like(mask)
                    cv2.bitwise_or(mask_union,mask,mask_union)

                contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if DEBUGGING:
                    h_image = hsv_image[:,:,0].copy()
                    self.h_img_pub.publish(bridge.cv2_to_imgmsg(h_image,"mono8"))
                
                
                for contour in contours:
                    x_min = int(contour[:,0,0].min()); x_max = int(contour[:,0,0].max()); x_mid = (x_max + x_min) // 2
                    y_min = int(contour[:,0,1].min()); y_max = int(contour[:,0,1].max()); y_mid = (y_max + y_min) // 2
                    
                    
                    bot_right = (x_max,y_max)
                    top_left = (x_min,y_min)
                    middle = (x_mid,y_mid)
                    pc = pc2.read_points(self.depth_msg,skip_nans=False,field_names=None,uvs=[middle])
                    #point = pc.next()
                    #print point
                    center_point_x = x_mid
                    center_point_y = (y_max + 2*y_min) // 3
                    center_point = (center_point_x,center_point_y)
                    z = np.nanmean(depth_image[center_point_y-10:center_point_y+10,center_point_x-10:center_point_x+10])
                    point = np.array(self.camera_model.projectPixelTo3dRay(center_point))
                    #print "distance from camera =", z
                    point = point * z
                    obj_img = bridge.cv2_to_imgmsg(
                            extract_object_image(middle,top_left,bot_right,rgb_image),
                            encoding="rgb8"
                        )
                    obj_cand_msg = PosAndImage()
                    obj_cand_msg.header.stamp = rospy.Time.now()
                    obj_cand_msg.header.frame_id = "camera_link" #We might need to change this to it's propper value
                    obj_cand_msg.pos.x = point[2]
                    obj_cand_msg.pos.y = - point[0]
                    obj_cand_msg.pos.z = - point[1]
                    obj_cand_msg.image = obj_img
                    color_msg = String_msg()
                    color_msg.data = color
                    obj_cand_msg.color = color_msg

                    print("x from camera coord=", point[2])
                    print("y from camera coord=", - point[0])
                    print("z from camera coord=", - point[1])

                    self.obj_cand_pub.publish(obj_cand_msg)
                     
                    if DEBUGGING:
                        cv2.drawContours(rgb_dbg,[contour],-1,color=color_2_rgb[color],thickness=-1)
                        cv2.rectangle(rgb_dbg,top_left,bot_right,color=(0,0,0),thickness=2)
                        cv2.circle(rgb_dbg,center_point,radius=5,color=(0,0,0),thickness=2)

                        self.dbg_object_pos.publish(obj_cand_msg.pos)
                        self.dbg_object_image.publish(obj_img)
                         
            if DEBUGGING:
                    
                ros_out_image = bridge.cv2_to_imgmsg(rgb_dbg,"rgb8")
                self.dbg_img_pub.publish(ros_out_image)
                self.h_mask_pub.publish(
                    bridge.cv2_to_imgmsg(mask_union,"mono8"))

        self._have_received_image = False
        self._have_received_depth = False
        

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
