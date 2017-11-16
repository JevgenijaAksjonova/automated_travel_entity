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
GOAL_ACHIEVED_TOPIC = "navigation/status"
MOTHER_WORKING_FRAME = "world_map"
ARM_PICKUP_SERVICE_NAME = "/arm/pickup"
ARM_MOVEMENT_COMPLETE_TOPIC = "/arm/done"

USING_PATH_PLANNING = False
USING_ARM = False
USING_VISION = True

color_2_rgb = {
    "green" : (0,255,0),
    "red" : (255,0,0),
    "blue_low" : (0,0,255),
    "blue" : (0,0,255),
    "yellow" : (0,255,255),
}

#Define verboseness of different parts

VISION_VERBOSE = False
DETECTION_VERBOSE = True