#! /usr/bin/env python
from __future__ import print_function
import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path("mother"))

import rospy
from rospy.service import ServiceException
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose, Vector3,Twist,PointStamped
from std_msgs.msg import Bool, String
from project_msgs.srv import global_path, exploration, global_pathRequest, explorationRequest
from project_msgs.msg import stop
from nav_msgs.msg import Odometry
from tf import TransformListener, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion
trans = TransformListener()
from camera.msg import PosAndImage
from camera.srv import recognizer
from visualization_msgs.msg import Marker
from ras_msgs.msg import RAS_Evidence
import numpy as np
from math import atan2
import yaml
from os import path
from maze import MazeMap, MazeObject, tf_transform_point_stamped, TRAP_CLASS_ID
from mother_settings import USING_VISION, OBJECT_CANDIDATES_TOPIC, GOAL_ACHIEVED_TOPIC, GOAL_POSE_TOPIC, ARM_MOVEMENT_COMPLETE_TOPIC, ODOMETRY_TOPIC, RECOGNIZER_SERVICE_NAME, USING_PATH_PLANNING, NAVIGATION_GOAL_TOPIC, NAVIGATION_EXPLORATION_TOPIC, NAVIGATION_STOP_TOPIC, USING_ARM, ARM_PICKUP_SERVICE_NAME, DETECTION_VERBOSE, MOTHER_WORKING_FRAME, ROUND, MAP_P_DECREASE,MAP_P_INCREASE,SAVE_PERIOD_SECS, MOTHER_STATE_FILE, RECOGNITION_MIN_P
from pprint import pprint

def call_srv(serviceHandle,request,max_attempts=float("inf"),retry_delay_secs = 5):
    #attempts = 0
    #while attempts < max_attempts:
    #    try:
    return serviceHandle(request)
    #    except ServiceException as se:
    #        rospy.logerr(se)
    #        rospy.sleep(rospy.Duration(secs=retry_delay_secs))

class Mother:

    odometry_msg = None
    classifying_obj = None
    i = 0
    arm_movement_success = None
    lifting_object = None
    object_classification_queue = []
    problem_with_path_following = False
    nav_goal_acchieved = True
    stop_info = stop()
    mode = "waiting_for_main_goal"

    def init_default_state(self):
            self.has_started = False
            self.maze_map = MazeMap(self.map_pub,MAP_P_INCREASE,MAP_P_DECREASE)
            self.exploration_completed = False

    def load_state(self):
        self.init_default_state()
        if path.isfile(MOTHER_STATE_FILE):
            with open(MOTHER_STATE_FILE,"r") as state_file:
                state_dict = yaml.load(state_file.read())
                self.has_started = state_dict["has_started"]
                self.exploration_completed = state_dict["exploration_completed"]
                print("loaded has_started = ",self.has_started)
                if self.has_started:
                    self.maze_map.load_maze_objs()
        else:
            print("No state file found")    
    def write_state(self):
        state_dict = {
            "has_started":self.has_started,
            "exploration_completed":self.exploration_completed,
        }
        with open(MOTHER_STATE_FILE,"w") as state_file:
            yaml.dump(state_dict,state_file)
        self.maze_map.save_maze_objs()

    def __init__(self):
        # A dictionary of all spotted objects.
        # form (x,y):"type"
        # where (x,y) are coordinates in the odom frame(shoudl this be map frame Jegvenja?) and
        # "type" is one of the object classes specified in RAS_EVIDENCE
        # before classification, "type" is "an_object"

        # Add your subscribers and publishers, services handels
        # and any other initialisation bellow

        #Publishers
        self.evidence_pub = rospy.Publisher(
            "camera/evidence", RAS_Evidence, queue_size=1)
        #self.navigation_goal_pub = rospy.Publisher(NAVIGATION_GOAL_TOPIC, Twist ,queue_size=1)
        self.speak_pub = rospy.Publisher("espeak/string", String, queue_size=1)

        self.trap_pub = rospy.Publisher("mother/trap",PointStamped,queue_size=1)

        self.map_pub = rospy.Publisher("mother/objects", Marker, queue_size=20)

        self.load_state()

        #Subscribers
        if USING_VISION:
            rospy.Subscriber(
                OBJECT_CANDIDATES_TOPIC,
                PosAndImage,
                callback=self._obj_cand_callback)
        
        rospy.Subscriber(
            NAVIGATION_EXPLORATION_STATUS_TOPIC,
            Bool,
            callback=self._exploration_status_callback)

        rospy.Subscriber(
            GOAL_POSE_TOPIC, PoseStamped, callback=self._goal_pose_callback)

        rospy.Subscriber(
            GOAL_ACHIEVED_TOPIC,
            Bool,
            callback=self._navigation_status_callback)

        rospy.Subscriber(
            ARM_MOVEMENT_COMPLETE_TOPIC,
            Bool,
            callback=self._arm_movement_complete_callback)

        rospy.Subscriber(
            ODOMETRY_TOPIC, Odometry, callback=self._odometry_callback)

        #Wait for required services to come online and service handles
        if USING_VISION:
            rospy.loginfo(
                "Waiting for service {0}".format(RECOGNIZER_SERVICE_NAME))
            rospy.wait_for_service(RECOGNIZER_SERVICE_NAME)
            self.recognizer_srv = rospy.ServiceProxy(
                RECOGNIZER_SERVICE_NAME, recognizer, persistent=True)

        if USING_PATH_PLANNING:
            rospy.loginfo(
                "Waiting for service {0}".format(NAVIGATION_GOAL_TOPIC))
            rospy.wait_for_service(NAVIGATION_GOAL_TOPIC)
            self.global_path_service = rospy.ServiceProxy(
                NAVIGATION_GOAL_TOPIC, global_path, persistent=True)
            #("after nave goal topic")
            if ROUND == 1:
                rospy.loginfo(
                    "Waiting for service {0}".format(NAVIGATION_EXPLORATION_TOPIC))
                rospy.wait_for_service(NAVIGATION_EXPLORATION_TOPIC)
                self.exploration_path_service = rospy.ServiceProxy(
                    NAVIGATION_EXPLORATION_TOPIC, exploration, persistent=True)
            rospy.Subscriber(
                NAVIGATION_STOP_TOPIC, stop, callback=self._navigation_stop_callback, queue_size=1)
            #print("after navigation stop topic")
            self.stop_pub = rospy.Publisher(NAVIGATION_STOP_TOPIC, stop,queue_size=1)

        if USING_ARM:
            rospy.loginfo(
                "Waiting for service {0}".format(ARM_PICKUP_SERVICE_NAME))
            rospy.wait_for_service(ARM_PICKUP_SERVICE_NAME)
            self.arm_pickup_srv = rospy.ServiceProxy(
                ARM_PICKUP_SERVICE_NAME, Point, persistent=True)

        #Other initialisations


    # Define your callbacks bellow like _obj_cand_callback.
    # The callback must return fast.

    def _odometry_callback(self, odom_msg):
        self.odometry = odom_msg

    def _arm_movement_complete_callback(self, success_msg):
        self.arm_movement_success = success_msg

    def _obj_cand_callback(self, obj_cand_msg):
        self.obj_cand_msg = obj_cand_msg
        self._handle_object_candidate_msg(obj_cand_msg)

    def _goal_pose_callback(self, goal_pose_msg):
        rospy.loginfo("goal pose callback")
        self.goal_pose = goal_pose_msg

    def _navigation_status_callback(self, status_msg):
        #rospy.loginfo("navigation status callback")
        status = status_msg.data
        if status:
            self.nav_goal_acchieved = True
        else:
            self.nav_goal_acchieved = False
            #rospy.loginfo("navigation status = false")
    
    def _exploration_status_callback(self,status_msg):
        self.exploration_completed = status_msg.data
        

    def _navigation_stop_callback(self, stop_msg):
        if stop_msg.stop and (self.mode == "following_an_exploration_path" or self.mode == "following_path_to_object_classification" or self.mode == "following_path_to_main_goal"):
            #if (self.mode != "handling_emergency_stop"):
            #rospy.loginfo("navigation stop callback")
            self.stop_info = stop_msg
            #self.mode = "handling_emergency_stop"
            if stop_msg.stop:
                if stop_msg.reason == 1:
                    pass
            #        rospy.loginfo("EMERGENCY STOP, LIDAR")
                elif stop_msg.reason == 2:
                    pass
             #       rospy.loginfo("EMERGENCY STOP, DEPTH")
                elif stop_msg.reason == 3:
                    pass
              #      rospy.loginfo("EMERGENCY STOP, LPP: NO WAY")
                elif stop_msg.reason == 4:
                    pass
               #     rospy.loginfo("EMERGENCY STOP, DEVIATION FROM A PATH")
                else:
                    pass
                #    rospy.loginfo("EMERGENCY STOP, REASON NOT SPECIFIED")
            # response
            msg = stop()
            msg.stop = False
            if stop_msg.reason == 1 or stop_msg.reason == 2 or stop_msg.reason == 3:
                msg.replan = False
                msg.rollback = True
            elif stop_msg.reason == 4:
                msg.replan = True
                msg.rollback = False
            else :
                msg.replan = True
                msg.rollback = True
            self.stop_pub.publish(msg)

    @property
    def pos(self):
        msg = PointStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = rospy.Time.now()
        msg_new = tf_transform_point_stamped(msg)
        pos = None
        i = 0
        while pos is None and i < 50:
            if msg_new is not None:
                pos = msg_new.point
                return np.r_[pos.x, pos.y]
            else:
                return None

    def _handle_object_candidate_msg(self, obj_cand_msg):
        try:
            obj_cand = MazeObject(obj_cand_msg)
            if DETECTION_VERBOSE:
                rospy.loginfo("successfully created maze object")
        except ExtrapolationException:
            if DETECTION_VERBOSE:
                rospy.loginfo("Could not transform because tf sucks")
            return
        except ValueError as nan_coord:
            if DETECTION_VERBOSE:
                rospy.logwarn(nan_coord)
            return
        self.maze_map.add_object(obj_cand)
        if obj_cand.class_id == TRAP_CLASS_ID:
            trap_msg = PointStamped()
            trap_msg.header.frame_id = MOTHER_WORKING_FRAME
            trap_msg.header.stamp = rospy.Time.now()
            trap_msg.point = Point(obj_cand.pos[0],obj_cand.pos[1],obj_cand.height)
            self.trap_pub.publish(trap_msg)

    def go_to_twist(self,twist,distance_tol=0.05,angle_tol=0.1):
        if USING_PATH_PLANNING:
            self.nav_goal_acchieved = None
            request = global_pathRequest()
            request.pose = twist
            request.distanceTol = distance_tol
            request.angleTol = angle_tol
            response = call_srv(self.global_path_service,request)
            return response.path_found
        else:
            self.nav_goal_acchieved = True
            return True


    def go_to_pose(self, pose,distance_tol=0.05,angle_tol=0.1):
        #print("go pose = ", type(pose))
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
            request = global_pathRequest()
            request.pose.linear.x = pose.pose.position.x
            request.pose.linear.y = pose.pose.position.y
            request.distanceTol = distance_tol
            request.angleTol = angle_tol
            response = call_srv(self.global_path_service,request)
            return response.path_found
        else:
            self.nav_goal_acchieved = True
            return True

    def try_classify(self):
        rospy.loginfo("Trying to classify")
        print("---------------classifying object---------------")
        print(self.classifying_obj)

        if self.classifying_obj is not None:
            resp = call_srv(self.recognizer_srv,self.classifying_obj.image)
            rospy.loginfo("resp.probability = {0}".format(
                resp.probability.data))
            rospy.loginfo("resp.probability > {min_p} = {p}".format(p=
                resp.probability.data > RECOGNITION_MIN_P,min_p = RECOGNITION_MIN_P))
            rospy.loginfo("resp.class_name = {0}".format(resp.class_name.data))
            if resp.probability.data > RECOGNITION_MIN_P and self.classifying_obj.color.lower() in resp.class_name.data.lower():
                self.classifying_obj.classify(resp.class_name.data,resp.class_id.data)
                rospy.loginfo("returning true from try classify")
                return True
            return False

    def set_following_path_to_main_goal(self):
        if self.go_to_pose(self.goal_pose, 0.05):
            self.mode = "following_path_to_main_goal"
            rospy.loginfo("Following path to main goal")
        else:
            rospy.loginfo("Could not find path to given main goal")
            self.set_waiting_for_main_goal()

    def set_following_an_exploration_path(self):
        self.mode = "following_an_exploration_path"
        if USING_PATH_PLANNING:
            # send a command to generate and follow an exploration path
            request = explorationRequest()
            request.req = True
            print("calling exploration_path_service")
            response = call_srv(self.exploration_path_service,request)

    def set_waiting_for_main_goal(self):
        self.goal_pose = None
        self.mode = "waiting_for_main_goal"
        rospy.loginfo("Waiting for main goal")

    def set_following_path_to_object_classification(self, classifying_obj):
        
        classification_pose = classifying_obj.pose_stamped
        if self.go_to_pose(classification_pose,angle_tol=2 * np.pi):
            self.mode = "following_path_to_object_classification"
            self.classifying_obj = classifying_obj
        else:
            rospy.loginfo("Did not find any feasable path to the object")

    def set_turning_towards_object(self,classifying_obj):
        robot_pos = self.pos
        if robot_pos is None:
            rospy.loginfo("was not able to set turning_towards_object because robot_pos was None")
            return False
        [x,y] = classifying_obj.pos - robot_pos
        theta = atan2(y,x)
        print("Sending message to turn to angle ",theta)
        msg = Twist()
        msg.angular = Vector3(0,0,theta)
        msg.linear = Vector3(robot_pos[0],robot_pos[1],0)
        if self.go_to_twist(msg,distance_tol=100000,angle_tol=0.1):
            self.mode = "turning_towards_object"
            self.classifying_obj = classifying_obj
            rospy.loginfo("We were able to find a way to turn")
            return True
        else:
            rospy.loginfo("Was not able to find way to turn to that object")
            return False

    def turning_towards_object_update(self):
        if ROUND == 1:
            activate_next_state = self.set_following_an_exploration_path
        elif ROUND == 0:
            activate_next_state = self.set_following_path_to_main_goal
        else:
            raise NotImplementedError()
        
        if self.nav_goal_acchieved is not None:
            if self.nav_goal_acchieved:
                rospy.Rate(1).sleep() 
                if self.try_classify():
                    classification_msg = "classified {label} at x = {x} and y = {y} in {frame} frame".format(
                        x=np.round(self.classifying_obj.pos[0], 2),
                        y=np.round(self.classifying_obj.pos[1], 2),
                        label=self.classifying_obj.class_label,
                        frame=MOTHER_WORKING_FRAME)
                    msg = String()
                    msg.data = classification_msg
                    rospy.loginfo(classification_msg)
                    self.speak_pub.publish(msg)
                    self.evidence_pub.publish(
                        self.classifying_obj.get_evidence_msg())
                    if "Cube" in self.classifying_obj.class_label:
                        rospy.loginfo("Object {0} is liftable".format(
                            self.classifying_obj.class_label))
                        self.set_lift_up_object(self.classifying_obj,activate_next_state)
                    else:
                        rospy.loginfo("{0} is not liftable".format(
                            self.classifying_obj.class_label))
                        activate_next_state()
                            
                    self.classifying_obj = None
                else:
                    activate_next_state()
                    self.classifying_obj.classification_attempts += 1
                    self.classifying_obj = None
            else:
                activate_next_state()

    def set_lift_up_object(self, lifting_obj,activate_next_state):
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
            activate_next_state()

    #Returns true if the mother mode has been changed.
    def classify_if_close(self,set_continue_state):
        self.object_classification_queue = list(
            self.maze_map.get_unclassified_objects(robot_pos=self.pos,distance_thresh=1,max_classification_attempts=0))
        if len(self.object_classification_queue) > 0:
            classifying_obj = self.object_classification_queue.pop()
            #print("setting turning towards object")
            if not self.set_turning_towards_object(classifying_obj):
                set_continue_state()
            else
                return True
                #self.set_following_an_exploration_path()
        return False
        
    # Main mother loop
    def mother_forever(self, rate=5):
        self.rate = rospy.Rate(rate)
        self.rate.sleep()

        self.set_waiting_for_main_goal()
        rospy.loginfo("Entering mother loop")
    
        last_save_secs = rospy.Time.now().to_sec()
        
        
        while not rospy.is_shutdown():

            if self.mode == "waiting_for_main_goal":
                if self.goal_pose is not None or self.has_started:
                    while True:
                        self.initial_pose = self.pos
                        if self.initial_pose is not None:
                            break
                    #robot_pos = self.pos 
                    #if robot_pos is not None:
                        #msg = Twist()
                        #msg.angular = Vector3(0,0,1.57)
                        #msg.linear = Vector3(robot_pos[0],robot_pos[1],0)   
                        #print("go_to_twist =",self.go_to_twist(msg,distance_tol=100000))
                    if ROUND == 1:
                        rospy.loginfo("Following an exploration path")
                        self.has_started = True
                        self.set_following_an_exploration_path()
                    
                    else:
                        rospy.loginfo("Main goal received")
                        self.set_following_path_to_main_goal()

            elif self.mode == "following_path_to_main_goal":
                changed_mode = self.classify_if_close(self.set_following_path_to_main_goal)
                if self.nav_goal_acchieved and not changed_mode:
                    self.set_waiting_for_main_goal()

            elif self.mode == "following_an_exploration_path":
                changed_mode = self.classify_if_close(self.set_following_an_exploration_path)
                if self.exploration_completed and not changed_mode:
                    self.exploration_completed = False
                    self.goal_pose = self.initial_pose
                    self.set_following_path_to_main_goal()


            elif self.mode == "following_path_to_object_classification":
                if self.nav_goal_acchieved:
                    if not self.set_turning_towards_object(self.classifying_obj):
                        self.set_following_an_exploration_path()
                    
            elif self.mode == "turning_towards_object":
                self.turning_towards_object_update()

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

            elif self.mode == "handling_emergency_stop":
                pass
                #rospy.loginfo("Handling emergency stop")
            else:
                raise Exception('invalid mode: \"' + str(self.mode) + "\"")

            #rospy.loginfo("mother iter {i}\n".format(i = self.i))
            #rospy.loginfo("\tClassification queue = {0}".format(self.object_classification_queue))
            #rospy.loginfo("\tclassifying object = {0}".format(self.classifying_obj ))
            rospy.loginfo("\tdetected objects = \n{0}".format(self.maze_map.maze_objects))
            rospy.loginfo("\tNew Mother loop, mode = \"{0}\"".format(self.mode))
            #rospy.loginfo("\tGoal pos = {goal}".format(goal = self.goal_pose))
            #rospy.loginfo("\tLifting object = {lifting}".format(lifting=self.lifting_object))
            self.maze_map.update(exclude_set={self.classifying_obj})

            if rospy.Time.now().to_sec() - last_save_secs > SAVE_PERIOD_SECS:
                self.write_state()
            self.rate.sleep()
            

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
