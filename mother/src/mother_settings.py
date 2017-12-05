#! /usr/bin/env python
from __future__ import print_function

import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0,rospack.get_path("mother"))

RECOGNIZER_SERVICE_NAME = "/camera/recognizer"
OBJECT_CANDIDATES_TOPIC = "/camera/object_candidates"
GOAL_POSE_TOPIC = "/move_base_simple/goal"
NAVIGATION_GOAL_TOPIC = "navigation/set_the_goal"
NAVIGATION_EXPLORATION_TOPIC = "navigation/exploration_path"
NAVIGATION_EXPLORATION_STATUS_TOPIC = "navigation/exploration_status"
NAVIGATION_STOP_TOPIC = "navigation/obstacles"
GOAL_ACHIEVED_TOPIC = "navigation/status"
MOTHER_WORKING_FRAME = "odom"
ARM_PICKUP_SERVICE_NAME = "uarm/arm_controller_service"
ARM_MOVEMENT_COMPLETE_TOPIC = "/arm/done"
ODOMETRY_TOPIC = "/odometry_node/odom"
NAVIGATION_DISTANCE_TOPIC = "navigation/distance"
USING_PATH_PLANNING = True
USING_ARM = True
USING_VISION = True

TIME_R1 = 300
TIME_R2 = 180
TIME_TO_GO_BACK = 30

ROUND = 1 # 1 - the first raund of the contest, 2 - the second raund of the contest

color_2_rgb = {
    "green": (0, 255, 0),
    "red": (255, 0, 0),
    "blue": (0, 0, 255),
    "yellow": (255, 255, 0),
    "purple": (128, 0, 128),
    "orange": (255, 165, 0),
    "gray": (128,128,128),
    "error": (128, 128, 128),
}

shape_2_allowed_colors = {
    "Cube": ["blue","green","yellow"],
    "Hollow Cross": ["orange","purple"],
    "Hollow Cube": ["red","green"],
    "Hollow Cylinder":["green","red"],
    "Hollow Triangle":["blue"],
    "Nothing":["green","red","blue","yellow","purple","orange"],
    "Sphere":["yellow","red"],
    "Star":["purple","orange"],
}

liftable_shapes = {
    "Cube"
}

#Define verboseness of different parts

VISION_VERBOSE = True
DETECTION_VERBOSE = False
MAP_P_INCREASE = 0.05
MAP_P_DECREASE = 0.005
SAVE_PERIOD_SECS = 3
MOTHER_STATE_FILE = "mother_state.yaml"
RECOGNITION_MIN_P = 0.65
CLASSIFYING_BASED_ON_COLOR = False
ARM_LIFT_ACCEPT_THRESH = 0.03
