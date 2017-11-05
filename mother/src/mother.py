#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
from geometry_msgs.msg import Pose2D, PoseStamped, PointStamped, Quaternion, Point, Pose, Twist, TransformStamped
from std_msgs.msg import Bool, String
from project_msgs.srv import global_path
from tf import TransformListener, ExtrapolationException
from tf.transformations import quaternion_from_euler, vector_norm
trans = TransformListener()
from camera.msg import PosAndImage
from camera.srv import recognizer, recognizerRequest, recognizerResponse
from sensor_msgs.msg import Image

from ras_msgs.msg import RAS_Evidence

#from ras_msgs.msg import RAS_Evidence

#import random

import numpy as np

RECOGNIZER_SERVICE_NAME = "/camera/recognizer"
OBJECT_CANDIDATES_TOPIC = "/camera/object_candidates"
GOAL_POSE_TOPIC = "/move_base_simple/goal"
NAVIGATION_GOAL_TOPIC = "navigation/set_the_goal"
GOAL_ACHIEVED_TOPIC = "navigation/status"
MOTHER_WORKING_FRAME = "odom"  #REMEMBER TO CHANGE TO MAP IN THE END!!!!
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

#Testing variables for disabling parts of the mother

USING_PATH_PLANNING = False
USING_ARM = False

#Define verboseness of different parts
RECOGNITION_VERBOSE = True
DETECTION_VERBOSE = False

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

class MazeObject:

    def __init__(self,obj_cand_msg,class_label="an_object",class_id=-1):
        obj_cand_point_msg = PointStamped()
        obj_cand_point_msg.header.frame_id = obj_cand_msg.header.frame_id
        obj_cand_point_msg.header.stamp = obj_cand_msg.header.stamp
        obj_cand_point_msg.point = obj_cand_msg.pos
        ros_sucks = True
        start_t = rospy.Time.now().nsecs
        obj_cand_msg_new = None
        i = 0
        while ros_sucks and i <100:
            try:
                obj_cand_msg_new = trans.transformPoint(MOTHER_WORKING_FRAME,obj_cand_point_msg)
                ros_didnt_suck = False
            except ExtrapolationException as rosfuck:
                #rospy.loginfo("i = {i}, stamp = {stamp}".format(
                #    i=i,stamp=obj_cand_point_msg.header.stamp))
                #rospy.loginfo("TF problem: {0}".format(rosfuck))
                pass
            i+=1
        if obj_cand_msg_new is not None:
            obj_cand_msg.header = obj_cand_msg_new.header
            obj_cand_msg.pos = obj_cand_msg_new.point
            self.pos = np.r_[obj_cand_msg.pos.x,obj_cand_msg.pos.y]
            if np.any(np.isnan(self.pos)):
                self.pos = None

            self.class_label = class_label
            self.class_id = class_id
            self.image = obj_cand_msg.image
        else:
            raise ExtrapolationException

    def is_close(self,other,tol=0.03):
        return self.point_is_close(other.pos,tol=tol)

    def point_is_close(self,point,tol=0.1):
        point = np.asarray(point)
        return tol > np.linalg.norm(self.pos - point)

    def get_evidence_msg(self):
        msg = RAS_Evidence()
        msg.stamp = rospy.Time.now()
        msg.image_evidence = self.image
        msg.object_id = self.class_label
        pos_trans = TransformStamped()
        pos_trans.transform.translation.x = self.pos[0]
        pos_trans.transform.translation.y = self.pos[1]
        msg.object_location = pos_trans
        return msg

    def __str__(self):
        return "{label} at {pos}".format(label=self.class_label,pos=self.pos)

    def __repr__(self):
        return self.__str__()
class Mother:
    
    def __init__(self): 
            #Initialisation of expressions not dependent on imput arguments or rospy.
        self.problem_with_path_following = False
        self.nav_goal_acchieved = True

        self.detected_objects = []
        self.mode = "waiting_for_main_goal"

        # A dictionary of all spotted objects.
        # form (x,y):"type"
        # where (x,y) are coordinates in the odom frame(shoudl this be map frame Jegvenja?) and 
        # "type" is one of the object classes specified in RAS_EVIDENCE
        # before classification, "type" is "an_object"

        self.object_classification_queue = []
        # Add your subscribers and publishers, services handels
        # and any other initialisation bellow
        
        #Subscribers
        rospy.Subscriber(OBJECT_CANDIDATES_TOPIC,PosAndImage,
            callback=self._obj_cand_callback)
        
        rospy.Subscriber(GOAL_POSE_TOPIC,PoseStamped,
            callback=self._goal_pose_callback)

        rospy.Subscriber(GOAL_ACHIEVED_TOPIC,Bool,
            callback=self._navigation_status_callback)

        #Publishers
        self.evidence_pub = rospy.Publisher("camera/evidence",RAS_Evidence,queue_size=1)
        #self.navigation_goal_pub = rospy.Publisher(NAVIGATION_GOAL_TOPIC, Twist ,queue_size=1)
        self.speak_pub = rospy.Publisher("espeak/string",String,queue_size=1)

        #Wait for required services to come online
        rospy.loginfo("Waiting for service {0}".format(RECOGNIZER_SERVICE_NAME))
        rospy.wait_for_service(RECOGNIZER_SERVICE_NAME)

        #Service handles
        self.recognizer_srv = rospy.ServiceProxy(RECOGNIZER_SERVICE_NAME,recognizer,persistent=True)
	self.global_path_service = rospy.ServiceProxy(NAVIGATION_GOAL_TOPIC, global_path , persistent=True)
        
        #Other initialisations
        self.classifying_obj = None

    # Define your callbacks bellow like _obj_cand_callback.
    # The callback must return fast.
    
    def _obj_cand_callback(self,obj_cand_msg):
        self.obj_cand_msg = obj_cand_msg
        self._handle_object_candidate_msg(obj_cand_msg)

    def _goal_pose_callback(self,goal_pose_msg):
        rospy.loginfo("goal pose callback")
        self.goal_pose = goal_pose_msg

    def object_at_pos(self,pos,tol=.1):
        for obj_id, obj in enumerate(self.detected_objects):
            if obj.point_is_close(pos):
                return obj_id,obj

    def _navigation_status_callback(self,status_msg):
        rospy.loinfo("navigation status callback")
        status = status_msg.data
        if status:
            self.nav_goal_acchieved = True

    def _handle_object_candidate_msg(self,obj_cand_msg):
        #Round
        #TODO: Define grid in a better manner, now we have 1 dm resolution.
        try:
            obj_cand = MazeObject(obj_cand_msg)
        except ExtrapolationException as tfuck:
            if DETECTION_VERBOSE:
                rospy.loginfo("Could not transform because tf sucks")
            return
        if obj_cand.pos is not None :
            object_at_pos_and_id = self.object_at_pos(obj_cand.pos)
            if object_at_pos_and_id is not None:
                obj_id,obj_at_pos = object_at_pos_and_id
                if DETECTION_VERBOSE:
                    rospy.loginfo("new observation obj id {id} of object at {pos} ".format(id=obj_id, pos=obj_at_pos.pos))
                #New obeservation of obj => Update position measurment and picture to latest
                obj_at_pos.pos = obj_cand.pos
                obj_at_pos.image = obj_cand.image
            else:
                if DETECTION_VERBOSE:
                    rospy.loginfo("observation of new object at {0}".format(obj_cand.pos))
                #No previous observation at location 
                self.detected_objects.insert(0,obj_cand)
                self.object_classification_queue.append(obj_cand)
        else:
            if DETECTION_VERBOSE:
                rospy.loginfo("Invalid object position received. Ignoring")
    
    @property
    def robot_pose(self):

        pose = PoseStamped()
        
        pose.header.frame_id="/base_link"
        pose.header.stamp = rospy.Time.now()

        pos = Point()
        pos.x = 0; pos.y = 0; pos.z = 0
        pose.pose.position = pos
        pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,0))
        return trans.transformPose(MOTHER_WORKING_FRAME,pose).pose

    def get_pos_to_classify_object(self,object_pos):
        #TODO: Find resonable way of getting into position
        #current_pose = self.robot_pose
        #object_pos_msg = point_to_msg(*object_pos)

        #for i in range(100):

            #Sample point on circle
            #angle_sample = random.uniform(0,np.pi * 2)
            #radius = .2 #robot distance from object
            #x_sample = np.cos(angle_sample) * radius
            #y_sample = np.sin(angle_sample) * radius
            
            #Move circle to perspective of object
            #x_sample = object_pos.x + x_sample
            #y_sample = object_pos.y + y_sample

            #pose = pose_to_msg(x_sample,y_sample,theta)
            
            # Further optimization, score different positions and select a good one.
            # This would also enable batching requests to a map server
            # This whole function should problably be implemented at some map server 
            # well well.
            #if self.possible_pose(pose) and self.line_of_sight(pose.point,object_pos_msg):
            #    return pose
        return None

        pose = Pose()
        pose.x = object_pos[0]
        pose.y = object_pos[1]
        pose.z = 0
        pose.orientation = Quaternion(*quaternion_from_euler)
        return pose

    def go_to_pose(self,pose):

        self.nav_goal_acchieved = False

        #Jegvenja
        #Set goal and plan here.
        #If no fesable path was found, return false, else true
        #Using a ros service is probably right in this case.

        # When the nav goad ls achieved and the robot has stoped,
        # Set self.nav_goal_acchieved = True, in the apropriate callback.

        # If the path following fails call self.set_following_path_to_main_goal()
        # if not already following in that state, otherwise set self.set_waiting_for_main_goal()
        srv = global_path()
        srv.pose.linear.x = pose.position.x
        srv.pose.linear.y = pose.position.y
        srv.pose.angular.x = 1.57
        if global_path_service.call(srv):
            return srv.response.path_found
        else:
	        return False

    def try_classify(self):
        rospy.loginfo("Trying to classify") 
        if self.classifying_obj is not None:
            resp = self.recognizer_srv(self.classifying_obj.image)
            rospy.loginfo("resp.probability = {0}".format(resp.probability.data))
            rospy.loginfo("resp.probability > .95 = {0}".format(resp.probability.data > .95))
            if resp.probability.data > .95:
                
                self.classifying_obj.class_label = resp.class_name.data
                self.classifying_obj.class_id = resp.class_id.data
                rospy.loginfo("returning tru from try classify")
                return True
            return False
                

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
    
    def set_following_path_to_object_classification(self,classifying_obj):
        
        classification_pose = pose_to_msg(
            classifying_obj.pos[0],classifying_obj.pos[1],0)
        
        if self.go_to_pose(classification_pose):
            self.mode = "following_path_to_object_classification"
            self.classifying_obj = classifying_obj
        else:
            rospy.loginfo("Did not find any feasable path to the object")

    def set_lift_up_object(self,object_pos):
        if USING_ARM:
            if self.can_lift_object():
                rospy.log("lifting object at {0}".format(object_pos))
                self.mode = "lift_up_object"
            else:
                rospy.log("can't lift object because...")
        else:
            self.set_following_path_to_main_goal()

    # Main mother loop
    def mother_forever(self,rate=1):
        rate = rospy.Rate(rate)
        rate.sleep()

        self.set_waiting_for_main_goal()
        rospy.loginfo("Entering mother loop")
        
        while not rospy.is_shutdown():
            
            if self.mode == "waiting_for_main_goal":
                if self.goal_pose is not None:
                    rospy.loginfo("Main goal received")
                    self.set_following_path_to_main_goal()
            
            elif self.mode == "following_path_to_main_goal":
            
                if len(self.object_classification_queue) > 0:
                    classifying_obj = self.object_classification_queue.pop()
                    self.set_following_path_to_object_classification(classifying_obj)
            
            elif self.mode == "following_path_to_object_classification":

                if self.nav_goal_acchieved:
                    if self.try_classify():
                        classification_msg = "classified {label} at x = {x} and y = {y} in {frame} frame".format(
                            x = np.round(self.classifying_obj.pos[0],2), y = np.round(self.classifying_obj.pos[1],2),
                            label = self.classifying_obj.class_label,frame=MOTHER_WORKING_FRAME)

                        rospy.loginfo(classification_msg)
                        self.speak_pub.publish(classification_msg)
                        self.evidence_pub.publish(self.classifying_obj.get_evidence_msg())
                        if "Cube" in self.classifying_obj.class_label:
                            self.set_lift_up_object(classifying_obj)
                        else:
                            rospy.loginfo("{0} is not liftable".format(self.classifying_obj.class_label))
                            self.set_following_path_to_main_goal()
                            self.classifying_obj = None
                    else:
                        self.set_following_path_to_main_goal()
                        self.classifying_obj = None 

            elif self.mode == "lift_object":
                # Enyu do your stuff, not sure what needs to be done to lift something.
                # The object you should lift up exists int the classifying_obj variable.
                # That pos is not very exact though so one probably needs to hope that the
                #  object candidate we can see the object at classifying_obj for now.
                self.set_following_path_to_main_goal()
                self.classifying_obj = None
            else:
                raise Exception('invalid mode: \"' + str(self.mode) + "\"")
            
            rate.sleep()
            rospy.loginfo("Classification queue = {0}".format(self.object_classification_queue))
            
            rospy.loginfo("classifying object = {0}".format(self.classifying_obj ))
            rospy.loginfo("detected objects = {0}".format(self.detected_objects))
            rospy.loginfo("New Mother loop, mode = \"{0}\"".format(self.mode))
            
def main():        
    rospy.init_node("recognizer_server")    
    m = Mother()
    m.mother_forever()

if __name__ == "__main__":
    try:
        print("Hllow")
        main()
    except rospy.ROSInterruptException:
        pass
