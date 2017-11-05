#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion, Point, Pose
from tf import TransformerROS
from tf.transformations import quaternion_from_euler, vector_norm
trans = TransformerROS()
from camera.msg import PosAndImage
from camera.srv import recognizer, recognizerRequest, recognizerResponse
from sensor_msgs.msg import Image

from mother.msg import RAS_Evidence

#from ras_msgs.msg import RAS_Evidence

#import random

import numpy as np
RECOGNIZER_SERVICE_NAME = "/camera/recognizer"
OBJECT_CANDIDATES_TOPIC = "/camera/object_candidates"
GOAL_POSE_TOPIC = "/mother/goal_pos"
BRAIN_WORKING_FRAME = "/odom"
#GENERAL INFO FOR THE PYTHON NOVICE
# I have tried to put helpfull comments here for the people that ar not so used to python.
# most of the stuff should be self explanatory, and you can probably follow the
# pattern that allready exists mostly, but bellow are some general tips.
#
# _function_name is convention for a private function.
# function_name is convention for a public function.
#
# ros fucntions can't be called in function headders, since they are called at module load time and not when called.
#
# TIP FOR CODING IN PYTHON WITH ROS.
# VS CODE with the most downloaded python addon will give decent autocomplete suggestions even when using ros libs (ctrl+space).

def pose_to_msg(x,y,theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = quaternion_from_euler(0,0,theta)
    return pose

def point_to_msg(x,y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0
    return point

class Mother:
    
    #Initialisation of expressions not dependent on imput arguments or rospy.
    problem_with_path_following = False
    nav_goal_acchieved = True

    detected_objects = dict()
    mode = "waiting_for_main_goal"

    # A dictionary of all spotted objects.
    # form (x,y):"type"
    # where (x,y) are coordinates in the odom frame(shoudl this be map frame Jegvenja?) and 
    # "type" is one of the object classes specified in RAS_EVIDENCE
    # before classification, "type" is "an_object"

    object_classification_queue = []

    def __init__(self):
        # Add your subscribers and publishers, services handels
        # and any other initialisation bellow
        
        #Subscribers
        rospy.Subscriber(OBJECT_CANDIDATES_TOPIC,PosAndImage,
            callback=self._obj_cand_callback)
        
        rospy.Subscriber(GOAL_POSE_TOPIC,Pose2D,
            callable=self._goal_pose_callback)

        #Publishers
        self.evidence_pub = rospy.Publisher("evidence_publisher",RAS_Evidence,queue_size=1)
        
        
        #Wait for required services to come online
        rospy.wait_for_service(RECOGNIZER_SERVICE_NAME)

        #Service handles
        self.recognizer_srv = rospy.ServiceProxy(RECOGNIZER_SERVICE_NAME,recognizer,persistent=True)
        
        #Other initialisations


    # Define your callbacks bellow like _obj_cand_callback.
    # The callback must return fast.
    
    def _obj_cand_callback(self,obj_cand_msg):
        self.obj_cand_msg = obj_cand_msg
        self._handle_object_candidate_msg(obj_cand_msg)

    def _goal_pose_callback(self,goal_pose_msg):
        self.goal_pose = goal_pose_msg


    def _handle_object_candidate_msg(self,obj_cand_msg):
        obj_cand_msg = trans.transformPoint(BRAIN_WORKING_FRAME,obj_cand_msg.pos)
        #Round
        #TODO: Define grid in a better manner, now we have 1 dm resolution.
        pos = np.round(np.r_[obj_cand_msg.point.x,obj_cand_msg.point.y],decimals=1)
        pos = (pos[0],pos[1])
        if pos not in self.detected_objects:
            self.detected_objects[pos] = "an_object"
            self.object_classification_queue.append(pos)
        self.last_obj_cand = obj_cand_msg
    
    @property
    def robot_pose(self):

        pose = PoseStamped()
        
        pose.header.frame_id="/base_link"
        pose.header.stamp = rospy.Time.now()

        pos = Point()
        pos.x = 0; pos.y = 0; pos.z = 0
        pose.pose.position = pos
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,0))
        trans.transformP
        return trans.transformPose(BRAIN_WORKING_FRAME,pose).pose

    def get_pos_to_classify_object(self,object_pos):
        #TODO: Find resonable way of getting into position
        current_pose = self.robot_pose
        object_pos_msg = point_to_msg(*object_pos)

        for i in range(100):

            #Sample point on circle
            angle_sample = random.uniform(0,np.pi * 2)
            radius = .2 #robot distance from object
            x_sample = np.cos(angle_sample) * radius
            y_sample = np.sin(angle_sample) * radius
            
            #Move circle to perspective of object
            x_sample = object_pos.x + x_sample
            y_sample = object_pos.y + y_sample

            pose = pose_to_msg(x_sample,y_sample,theta)
            
            # Further optimization, score different positions and select a good one.
            # This would also enable batching requests to a map server
            # This whole function should problably be implemented at some map server 
            # well well.
            if self.possible_pose(pose) and self.line_of_sight(pose.point,object_pos_msg):
                return pose
        return None
        



        pose = Pose()
        pose.x = object_pos[0]
        pose.y = object_pos[1]
        pose.z = 0
        pose.orientation = Quaternion(*quaternion_from_euler)
        return pose

    def line_of_sight(self,point_1,point_2):
        #Check wether there is a clear line of sight between the two points
        #Map people, please fill in. Can probably ignore right now
        return True

    def go_to_pose(self,pose):
        self.nav_goal_acchieved = False
        self.path_following_failed = False

        #Jegvenja
        #Set goal and plan here.
        #If no fesable path was found, return false, else true
        #Using a ros service is probably right in this case.

        # When the nav goad ls achieved and the robot has stoped,
        # Set self.nav_goal_acchieved = True, in the apropriate callback.

        # If the path following fails
        # Set self.path_following_failed = True, in the apropriate callback.

        return True

    def possible_pose(self,pose):
        # Map guys, plz check, must be blocking. 
        # Somhow, take into account, pos of object we are trying to classify
        # This one would be really neat to have for ms3
        
        return True

    def try_classify(self,pos):
        #We are assuming that we now are in a position to see only one object
        #Which is of course unresonable, but we will deal with determening which of the
        #objects we are seeing that is at classification_object_pos later.
        
        if self.last_obj_cand is not None:
            resp = self.recognizer_srv(self.last_obj_cand.image)
            if resp.probability > .95:
                return resp.class_name,resp.class_id
        return None
        
    def set_following_path_to_main_goal(self):
        if self.go_to_pose(self.goal_pose):
            self.mode = "following_path_to_main_goal"
            rospy.loginfo("Following path to main goal")
        else:
            rospy.loginfo("Could not find path to given main goal")
            self.set_waiting_for_main_goal()

    def set_waiting_for_main_goal(self):
        self.goal_pose = None
        self.mode = "waiting_for_main_goal"
        rospy.loginfo("Waiting for main goal")
    
    def set_following_path_to_object_classification(self,classification_pose):
        if self.go_to_pose(classification_pose):
            self.mode = "set_following_path_to_object_classification"
        else:
            rospy.loginfo("Did not find any feasable path to the object")

    def set_lift_up_object(self,object_pos):
        if self.can_lift_object():
            rospy.log("lifting object at {0}".format(object_pos))
            self.mode = "lift_up_object"
        else:
            rospy.log("can't lift object because...")
    # Main mother loop
    def mother_forever(self,rate=.2):
        rate = rospy.Rate(rate)
        rate.sleep()

        self.set_waiting_for_main_goal()

        while not rospy.is_shutdown():
            
            if self.mode == "waiting_for_main_goal":
                if self.goal_pose is not None:
                    rospy.loginfo("Main goal received")
                    self.set_following_path_to_main_goal()
            
            elif self.mode == "following_path_to_main_goal":
            
                if len(self.object_classification_queue) > 0:
                    classification_object_pos = self.object_classification_queue.pop()
                    classification_pose = self.get_pos_to_classify_object(classification_object_pos)
                    if classification_pose is not None:
                        rospy.loginfo("Going to object at {0}".format(classification_object_pos))
                        self.set_following_path_to_object_classification(classification_pose)
                    else:
                        rospy.loginfo("no fesable position to observe object at {0} was found".format(classification_object_pos))
            
            elif self.mode == "following_path_to_object_classification":
            
                if self.nav_goal_acchieved:
                    object_class = self.try_classify(classification_object_pos)
                    if object_class is not None:
                        self.detected_objects[classification_object_pos] = object_class
                        rospy.loginfo("successfully classified object at {0} as {1}".format(
                            classification_object_pos,object_class))
                        if "Cube" in object_class:
                            self.set_lift_up_object(classification_object_pos)
                        else:
                            rospy.loginfo("{0} is not liftable")
                    else:
                        if not self.line_of_sight(self.robot_pose.position,point_to_msg(classification_object_pos)):
                            rospy.loginfo("could not classify object, possibly because it isn't line of sight")
                            self.object_classification_queue.append(classification_object_pos)
                        else:
                            rospy.loginfo("Failed to classify object")
                        
                        self.set_following_path_to_main_goal()
                        classification_object_pos = None
            
            elif self.mode == "lift_object":
                # Enyu do your stuff, not sure what needs to be done to lift something.
                # The object you should lift up exists int the classification_object_pos variable.
                # That pos is not very exact though so one probably needs to hope that the
                #  object candidate we can see the object at classification_object_pos for now.
                pass
            else:
                raise Exception('invalid mode: \"' + str(self.mode) + "\"")
            
            rate.sleep()

def main():        
    rospy.init_node("recognizer_server")    
    m = Mother()
    m.mother_forever()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass