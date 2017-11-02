#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
from tf import TransformerROS
trans = TransformerROS()
from camera.srv import recognizer
from camera.msg import PosAndImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from ras_msgs.msg import RAS_Evidence

import numpy as np

RECOGNIZER_SERVICE_NAME = "/camera/recognizer"
OBJECT_CANDIDATES_TOPIC = "/camera/object_candidates"
GOAL_POINT_TOPIC = "/mother/goal_pos"
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
        
        rospy.Subscriber(GOAL_POINT_TOPIC,Pose2D,
            callable=self._goal_point_callback)

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

    def _goal_point_callback(self,goal_point_msg):
        self.goal_point = goal_point_msg

    def _wait_for_goal(self,rate=2):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.has_received_main_goal_pose:
                return
            rate.sleep()

    def _handle_object_candidate_msg(self,obj_cand_msg):
        obj_cand_msg = trans.transformPoint("/odom",obj_cand_msg.pos)
        #Round
        #TODO: Define grid in a better manner, now we have 1 dm resolution.
        pos = np.round(np.r_[obj_cand_msg.point.x,obj_cand_msg.point.y],decimals=1)
        pos = (pos[0],pos[1])
        if pos not in self.detected_objects:
            self.detected_objects[pos] = "an_object"
            self.object_classification_queue.append(pos)

    def get_pos_to_classify_object(self,object_pos):
        #TODO: Find resonable way of getting into position
        
        pose = Pose2D()
        pose.x = object_pos[0]
        pose.y = object_pos[1]
        pose.theta = 0

        return pose

    def go_to_pose(self,pose):
        self.nav_goal_acchieved = False
        #Jegvenja
        #Set goal and plan here.
        
        #When the nav goad ls achieved and the robot has stoped,
        #Set self.nav_goal_acchieved = True in the apropriate callback.

    def try_classify(self,pos):
        #We are assuming that we now are in a position to see only one object
        #Which is of course unresonable, but we will deal with determening which of the
        #objects we are seeing that is at classification_object_pos later.

    # Main mother loop
    def mother_forever(self,rate=.2):
        rate = rospy.Rate(rate)
        rate.sleep()
        #Step 1: Listen for end goal coordinate       

        #Step 2: Path plan and execute path from here to point.

        # Jegvenja, add call to path finder here and execute path.
        # the goal is in self.goal_point and is a Pose2D.
        # Please change to any other type if required.
        # Allso, please update problem_with_path_following in callbacks as apropriate.

        while not rospy.is_shutdown():
            
            if self.mode == "waiting_for_main_goal":
                if self.has_received_main_goal_pose:
                    self.mode = "following_path_to_main_goal"
            
            elif self.mode == "following_path_to_main_goal":
                if len(self.object_classification_queue) > 0:
                    classification_object_pos = self.object_classification_queue.pop()
                    classification_pose = self.get_pos_to_classify_object(classification_object_pos)
                    self.go_to_pose(classification_pose) 
                    self.mode = "following_path_to_object_classification"
            elif self.mode == "following_path_to_object_classification":
                if self.nav_goal_acchieved:

                    self.try_classify(classification_object_pos)
            else:
                raise Exception('invalid mode: \"' + str(self.mode) + "\"")
            
            rate.sleep()

            if len(self.object_classification_queue) > 0:
                object_candidate_pos = self.object_classification_queue.pop()
                
                #Step 4: go to object position so that the arm can reach it.
                #Jegvenja, please add your stuff here to go to object_candidate_pos.
                #and update classification_goal_pose_achieved as appropriate.
                while not rospy.is_shutdown() and not self.classification_goal_pose_achieved:


                #Classify the object.

                #Then Enyu, please  the object upp
            elif self.problem_with_path_following:
                #Jegvenja, handle path following problem
                self.problem_with_path_following = False
                
            rate.sleep()
        #Step 3: loop and check if we detect any object_candidates.
        #If we detect a new object. Stop path execution.

       

        #Step 5: Classify the object.

        #Step 6: If the object is liftable, try to lift it up with the arm.

        #Step 7: Go back to Step 2.

        while not rospy.is_shutdown():
            print("loop")
            if self.has_received_obj_cand:
                print("encoding =",self.obj_cand_msg.image.encoding)
                try:
                    resp = self.recognizer_srv(self.obj_cand_msg.image)

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
    m = Mother()
    m.mother_forever()






if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass