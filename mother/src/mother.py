#! /usr/bin/env python
from __future__ import print_function

import rospy
import roslib
from geometry_msgs.msg import Pose2D, PoseStamped, PointStamped, Quaternion, Point, Pose, Twist, TransformStamped
from std_msgs.msg import Bool, String
from project_msgs.srv import global_path, global_pathRequest, global_pathResponse
from tf import TransformListener, ExtrapolationException
from tf.transformations import quaternion_from_euler, vector_norm
trans = TransformListener()
from camera.msg import PosAndImage
from camera.srv import recognizer, recognizerRequest, recognizerResponse
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from ras_msgs.msg import RAS_Evidence
import random
#from ras_msgs.msg import RAS_Evidence

#import random

import numpy as np

RECOGNIZER_SERVICE_NAME = "/camera/recognizer"
OBJECT_CANDIDATES_TOPIC = "/camera/object_candidates"
GOAL_POSE_TOPIC = "/move_base_simple/goal"
NAVIGATION_GOAL_TOPIC = "navigation/set_the_goal"
GOAL_ACHIEVED_TOPIC = "navigation/status"
MOTHER_WORKING_FRAME = "odom"  #REMEMBER TO CHANGE TO MAP IN THE END!!!!
ARM_PICKUP_SERVICE_NAME = "/arm/pickup"
ARM_MOVEMENT_COMPLETE_TOPIC = "/arm/done"
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
DETECTION_VERBOSE = False

def pose_to_msg(x,y,theta,z=0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    quat = quaternion_from_euler(0,0,theta)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def point_to_msg(x,y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0
    return point

def pose_to_msg_stamped(x,y,theta,z=0):
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose_to_msg(x,y,theta,z)
    pose_stamped.header.frame_id = MOTHER_WORKING_FRAME
    pose_stamped.header.stamp = rospy.Time.now()
    return pose_stamped

class MazeObject(object):

    n_maze_objects = 0

    def __init__(self,obj_cand_msg,class_label="an_object",class_id=-1,vis_pub=None):
        obj_cand_point_msg = PointStamped()
        obj_cand_point_msg.header.frame_id = obj_cand_msg.header.frame_id
        obj_cand_point_msg.header.stamp = obj_cand_msg.header.stamp
        obj_cand_point_msg.point = obj_cand_msg.pos
        self.color = obj_cand_msg.color.data
        ros_sucks = True
        obj_cand_msg_new = None
        i = 0
        while ros_sucks and i <100:
            try:
                obj_cand_msg_new = trans.transformPoint(MOTHER_WORKING_FRAME,obj_cand_point_msg)
                ros_didnt_suck = False
            except ExtrapolationException as rosfuck:
                pass
            i+=1

        if obj_cand_msg_new is None:
            raise ExtrapolationException

        obj_cand_msg.header = obj_cand_msg_new.header
        obj_cand_msg.pos = obj_cand_msg_new.point
        self._pos = np.r_[obj_cand_msg.pos.x,obj_cand_msg.pos.y]
        self.height = obj_cand_msg.pos.z
        
        if self.height < 0:
            if VISION_VERBOSE:
                rospy.logwarn("Object candidate with negative height")
        
        if np.any(np.isnan(self._pos)):
            raise ValueError("obj_cand_msg position contains nan values: {0}".format(self._pos))

        self.class_label = class_label
        self.class_id = class_id
        self.image = obj_cand_msg.image
        self.id = MazeObject.n_maze_objects
        MazeObject.n_maze_objects += 1
        self._marker = None
        self._vis_pub = None
        self.visulisation_publisher = vis_pub

    def is_close(self,other,tol=0.1):
        return self.point_is_close(other.pos,tol=tol)

    def point_is_close(self,point,tol=0.1):
        point = np.asarray(point)
        return tol > np.linalg.norm(self._pos - point)

    def is_close_and_same_color(self,other,tol=0.1):
        return self.is_close(other,tol) and self.color == other.color

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self,pos):
        self._pos = pos
        self._update_marker()

    @property
    def visulisation_publisher(self):
        return visulisation_publisher

    @visulisation_publisher.setter
    def visulisation_publisher(self,vis_pub):
        if vis_pub is None:
            self._remove_marker()
            self._vis_pub = None
        else:
            self._vis_pub = vis_pub
            self._add_marker()

    @property
    def pose_stamped(self):
        return pose_to_msg_stamped(self._pos[0],self._pos[1],0,self.height)

    def _remove_marker(self):
        if self._vis_pub is not None:
            self._marker.action = Marker.DELETE
            self._vis_pub.publish(self._marker)
            self._marker = None

    def _update_marker(self):
        if self._vis_pub is not None:
            print("Updating marker pos")
            pose_stmp = self.pose_stamped
            self._marker.header = pose_stmp.header
            self._marker.pose = pose_stmp.pose
            self._marker.action = Marker.MODIFY
            self._vis_pub.publish(self._marker)
        print("In update marker but no vis_pub")

    def _add_marker(self):
        if self._vis_pub is not None:
            self._marker = Marker()
            pose_stmp = self.pose_stamped
            self._marker.header = pose_stmp.header
            self._marker.pose = pose_stmp.pose
            self._marker.action = Marker.ADD
            self._marker.type = Marker.CUBE
            self._marker.scale.x = .02
            self._marker.scale.y = .02
            self._marker.scale.z = .02
            (r,g,b) = color_2_rgb[self.color]
            self._marker.color.r = r
            self._marker.color.g = g
            self._marker.color.b = b
            self._marker.color.a = 1
            self._marker.id = self.id
            self._marker.ns = "MazeObjects"
            self._marker.lifetime = rospy.Duration.from_sec(5)
            self._vis_pub.publish(self._marker)

    def get_evidence_msg(self):
        msg = RAS_Evidence()
        msg.stamp = rospy.Time.now()
        msg.image_evidence = self.image
        msg.object_id = self.class_label
        pos_trans = TransformStamped()
        pos_trans.transform.translation.x = self._pos[0]
        pos_trans.transform.translation.y = self._pos[1]
        msg.object_location = pos_trans
        return msg

    def __str__(self):
        return "{label} at {pos} id {id}".format(label=self.class_label,pos=self._pos,id=self.id)

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
        if USING_VISION:
            rospy.Subscriber(OBJECT_CANDIDATES_TOPIC,PosAndImage,
                callback=self._obj_cand_callback)
        
        rospy.Subscriber(GOAL_POSE_TOPIC,PoseStamped,
            callback=self._goal_pose_callback)

        rospy.Subscriber(GOAL_ACHIEVED_TOPIC,Bool,
            callback=self._navigation_status_callback)

        rospy.Subscriber(ARM_MOVEMENT_COMPLETE_TOPIC,Bool,
            callback=self._arm_movement_complete_callback)

        #Publishers
        self.evidence_pub = rospy.Publisher("camera/evidence",RAS_Evidence,queue_size=1)
        #self.navigation_goal_pub = rospy.Publisher(NAVIGATION_GOAL_TOPIC, Twist ,queue_size=1)
        self.speak_pub = rospy.Publisher("espeak/string",String,queue_size=1)

        self.map_pub = rospy.Publisher("mother/objects",Marker,queue_size=20)

        #Wait for required services to come online and service handles
        if USING_VISION:
            rospy.loginfo("Waiting for service {0}".format(RECOGNIZER_SERVICE_NAME))
            rospy.wait_for_service(RECOGNIZER_SERVICE_NAME)
            self.recognizer_srv = rospy.ServiceProxy(RECOGNIZER_SERVICE_NAME,recognizer,persistent=True)

        if USING_PATH_PLANNING:
            rospy.loginfo("Waiting for service {0}".format(NAVIGATION_GOAL_TOPIC))
            rospy.wait_for_service(NAVIGATION_GOAL_TOPIC)
            self.global_path_service = rospy.ServiceProxy(NAVIGATION_GOAL_TOPIC, global_path , persistent=True)
        
        if USING_ARM:
            rospy.loginfo("Waiting for service {0}".format(ARM_PICKUP_SERVICE_NAME))
            rospy.wait_for_service(ARM_PICKUP_SERVICE_NAME)
            self.arm_pickup_srv = rospy.ServiceProxy(ARM_PICKUP_SERVICE_NAME, Point , persistent=True)

        #Other initialisations
        self.classifying_obj = None
        self.i = 0
        self.arm_movement_success = None
        self.lifting_object = None
    # Define your callbacks bellow like _obj_cand_callback.
    # The callback must return fast.

    def _arm_movement_complete_callback(self,success_msg):
        self.arm_movement_success = success_msg
    
    def _obj_cand_callback(self,obj_cand_msg):
        self.obj_cand_msg = obj_cand_msg
        self._handle_object_candidate_msg(obj_cand_msg)

    def _goal_pose_callback(self,goal_pose_msg):
        rospy.loginfo("goal pose callback")
        self.goal_pose = goal_pose_msg

    def detected_objs_close(self,obj,tol=.1):
        close_objs = [detected_obj for detected_obj in self.detected_objects
            if obj.is_close_and_same_color(detected_obj)]
        return close_objs               


    def _navigation_status_callback(self,status_msg):
        rospy.loinfo("navigation status callback")
        status = status_msg.data
        if status:
            self.nav_goal_acchieved = True

    def _handle_object_candidate_msg(self,obj_cand_msg):
        try:
            obj_cand = MazeObject(obj_cand_msg)
        except ExtrapolationException as tfuck:
            if DETECTION_VERBOSE:
                rospy.loginfo("Could not transform because tf sucks")
            return
        except ValueError as nan_coord:
            if DETECTION_VERBOSE:
                rospy.logwarn(nan_coord)
            return
        close_objs = self.detected_objs_close(obj_cand)
        if len(close_objs) != 0:
            mean_height = np.mean([close_obj.height for close_obj in close_objs])
            mean_pos = np.mean([close_obj.pos for close_obj in close_objs],axis=0)
            close_images = [close_obj.image for close_obj in close_objs]
            largest_image_idx = np.argmax([close_image.height * close_image.width for close_image in close_images])
            image_large = close_images[largest_image_idx]
            obj_at_pos = close_objs.pop()
            #New obeservation of obj => Update position measurment and picture to latest
            obj_at_pos.height = mean_height
            obj_at_pos.pos = mean_pos
            obj_at_pos.image = image_large
            for close_obj in close_objs:
                close_obj.visulisation_publisher = None
                self.detected_objects.remove(close_obj)

            if DETECTION_VERBOSE:
                rospy.loginfo("new observation obj id {id} of object at {pos} ".format(id=obj_at_pos.id, pos=obj_at_pos.pos))
            
        else:
            if DETECTION_VERBOSE:
                rospy.loginfo("observation of new object at {0}".format(obj_cand._pos))
            #No previous observation at location 
            self.detected_objects.append(obj_cand)
            self.object_classification_queue.append(obj_cand)
            obj_cand.visulisation_publisher = self.map_pub
    
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
        print("go pose = ", type(pose))
        if USING_PATH_PLANNING:
            self.nav_goal_acchieved = False

            #Jegvenja
            #Set goal and plan here.
            #If no fesable path was found, return false, else true
            #Using a ros service is probably right in this case.

            # When the nav goad ls achieved and the robot has stoped,
            # Set self.nav_goal_acchieved = True, in the apropriate callback.

            # If the path following fails call self.set_following_path_to_main_goal()
            # if not already following in that state, otherwise set self.set_waiting_for_main_goal()
            request = Twist()
            request.linear.x = pose.pose.position.x
            request.linear.y = pose.pose.position.y
            request.angular.x = 1.57
            b = None
            response = self.global_path_service(request)
            return response.path_found
        else:
            self.nav_goal_acchieved = True
            return True

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
        
        classification_pose = classifying_obj.pose_stamped 
        if self.go_to_pose(classification_pose):
            self.mode = "following_path_to_object_classification"
            self.classifying_obj = classifying_obj
        else:
            rospy.loginfo("Did not find any feasable path to the object")

    def set_lift_up_object(self,lifting_obj):
        if USING_ARM:
            self.lifting_object = lifting_obj
            rospy.log("lifting object at {0}".format(lifting_obj))
            loc = Point()
            loc.x = self.classifying_obj.pos.x
            loc.y = self.classifying_obj.pos.y
            loc.z = self.classifying_obj.pos.z
            request_ok = self.arm_pickup_srv(loc)
            self.arm_movement_success = None
            if request_ok:
                rospy.loginfo("Arm request was ok")
            else:
                rospy.loginfo("requested position out of arm range")
            self.mode = "lift_up_object"
        else:
            self.set_following_path_to_main_goal()

    # Main mother loop
    def mother_forever(self,rate=10):
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
                    print("classifying object type = ", type(classifying_obj))
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
                            rospy.loginfo("Object {0} is liftable".format(self.classifying_obj.class_label))
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
                if self.arm_movement_success is not None:
                    if self.arm_movement_success:
                        self.set_following_path_to_main_goal()
                        self.arm_movement_success = None
                        self.lifting_object = None
                        rospy.loginfo("Arm movement success")
                    else:
                        rospy.loginfo("Arm movement failed")
            else:
                raise Exception('invalid mode: \"' + str(self.mode) + "\"")
            
            rospy.loginfo("mother iter {i}\n".format(i = self.i))
            rospy.loginfo("\tClassification queue = {0}".format(self.object_classification_queue))
            rospy.loginfo("\tclassifying object = {0}".format(self.classifying_obj ))
            rospy.loginfo("\tdetected objects = {0}".format(self.detected_objects))
            rospy.loginfo("\tNew Mother loop, mode = \"{0}\"".format(self.mode))
            rospy.loginfo("\tGoal pos = {goal}".format(goal = self.goal_pose))
            rospy.loginfo("\tLifting object = {lifting}".format(lifting=self.lifting_object))
            self.i += 1
            rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("recognizer_server")    
        m = Mother()
        m.mother_forever()
    except rospy.ROSInterruptException:
        if m is not None:
            if m.detected_objects is not None:
                for obj in m.detected_objects:
                    obj.visulisation_publisher = None
