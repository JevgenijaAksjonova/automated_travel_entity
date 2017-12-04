#! /usr/bin/env python
import numpy as np
import rospy
import rospkg
rospack = rospkg.RosPack()
import sys
from os import path
camera_base = rospack.get_path("camera")
sys.path.insert(0,path.join(camera_base,"src"))
import colorSegment
import yaml
import cv_bridge
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CompressedImage as CompMsg
import cv2
from colorSegment import color_segment_image
bridge = cv_bridge.CvBridge()

class the_class:

    def main(self):
        rospy.init_node("hsv_tuner")
        def image_callback(image_msg):
            self.image = bridge.imgmsg_to_cv2(image_msg)

        def depth_image_callback(depth_image_msg):
            self.depth_image = bridge.imgmsg_to_cv2(depth_image_msg)

        rospy.Subscriber("/camera/rgb/image_rect_color",ImageMsg,image_callback)
        rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect",ImageMsg,depth_image_callback)
        comp_img_pub = rospy.Publisher("/hsvdebug/compressed", CompMsg, queue_size=1)

        rospy.Rate(1).sleep()
        param_file_path = path.join(rospack.get_path("camera"),"param.yaml")
        with open(param_file_path,"r") as param_file:
            hsv_thresh = yaml.load(param_file)["camera"]["hsv_thresholds"]
        hsv_thresh = {color:(np.array(values["lower"]),np.array(values["upper"])) for color,values in hsv_thresh.items()}
        current_color = "red"
        while True:
            print("current color = ", current_color)
            print("Give color values as [h|s|v] [000 - 255] [000-255] or select new color from Blue, Green, Yellow, Purple, Orange, Red")
            input_strs = raw_input(">").lower().split(" ")
            if input_strs[0] in ["blue", "green", "yellow", "purple", "orange", "red"]:
                current_color = input_strs[0]

            elif input_strs[0] in "q":
                print("Closing down HSV calibration program")
                sys.exit(0)

            elif input_strs[0] in "w":

                save_param_file_path = path.join(rospack.get_path("camera"),"hsv_atrium_params.yaml")
                write_dict = {color:{
                    "lower":hsv_thresh[color][0].tolist(),
                    "upper":hsv_thresh[color][1].tolist()
                } for color, thresh in hsv_thresh.items()}
                print(write_dict)
                with open(save_param_file_path,"w") as save_file:
                    yaml.dump(write_dict, save_file, default_flow_style=True)

            elif len(input_strs[0]) != 0:
            
                if input_strs[0] not in "hsv" or len(input_strs[0]) != 1:
                    continue

                channel = {"h":0,
                        "s":1,
                        "v":2}[input_strs[0]]
                
                if len(input_strs[1]) < 1 or len(input_strs[2]) < 1 :
                    continue
                try:
                    lower = int(input_strs[1])
                    upper = int(input_strs[2])
                except ValueError:
                    continue
                lower_threshes,upper_threshes = hsv_thresh[current_color]
                lower_threshes[channel] = lower
                upper_threshes[channel] = upper
                hsv_thresh[current_color] = (lower_threshes,upper_threshes)
                if not (0 <= lower and lower <= 255) or not (0 <= upper and upper <= 255):
                    continue
                print("debug_simage.shape =", self.image.shape)
                _,_,debug_image = color_segment_image(self.image,self.depth_image,return_debug_image=True,apply_checks=False,hsv_thresholds=hsv_thresh)
                #cv2.imshow("wheeh",debug_image)
                compressed_img = bridge.cv2_to_compressed_imgmsg(debug_image)
                comp_img_pub.publish(compressed_img)
            else:
                print("Incorrect input: ")
                print(input_strs)
            hsv_thresh[current_color]
            rospy.Rate(1).sleep()




if __name__ == "__main__":
    the_class().main()