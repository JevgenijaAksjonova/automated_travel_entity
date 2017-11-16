#! /usr/bin/env python
from __future__ import print_function

import sys
import rospkg
rospack = rospkg.RosPack()
sys.path.insert(0,rospack.get_path("mother"))

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
from mother_settings import *
from mother_utils import *
import numpy as np


class MazeMap:
    #A general reprensentation of the robot map
    def __init__(self,visulisation_publisher,p_increse_rate,p_loss_rate):
        #Todo, supply map with walls and all to this
        self.maze_objects = set()
        self.visulisation_publisher = visulisation_publisher
        self.p_increse_rate = p_increse_rate
        self.p_loss_rate = p_loss_rate
    #When the map has been modified, one must allways update any external references
    #They may have disaperared, in witch case None is returned
    def update_maze_object_reference(self,obj):
        if obj in self.maze_objects:
            return obj #the reference is the same
        neighs = sorted(self._same_color_neighbors(obj),
            lambda obj_a, obj_b: np.linalg.norm(obj_a.pos - obj_b.pos))
        if len(neighs) > 0:
            return neighs[0] #Return closest neighbor
        
        return None #The object seems to be gone
    #returns the objects to be classified, sortend on proximity to robot_pos
    def get_unclassified_objects(self,robot_pos):
        return sorted([obj for obj in self.maze_objects if not obj.is_classified]
                ,lambda obj_a, obj_b: np.linalg.norm(obj_a.pos - robot_pos) - np.linalg.norm(obj_b.pos - robot_pos))

    # Add any type of object to the map.
    # Returns a reference to the object in the map 
    # as it may not be the same object as was added, eventhough it is equivilent
    def add_object(self,obj):
        if type(obj) is MazeObject:
            return self._add_maze_obj(obj)
        else:
            raise Exception("tried to add invalid object to map")

    #Update the map once every loop,
    #Removes inprobable objects from map
    def update(self):
        objs_to_remove = set()
        for obj in self.maze_objects:
            obj.p -= self.p_loss_rate
            obj._update_marker()
            if obj.p <= 0:
                obj.visulisation_publisher = None
                objs_to_remove.add(obj)
            print("------------------------- obj.p = {0} -----------------------".format(obj.p))
        self.maze_objects.difference_update(objs_to_remove)

    def _add_maze_obj(self,obj):
        neighs = self._same_color_neighbors(obj)
        obj.p = self.p_increse_rate
        obj.classified = False
        if len(neighs) > 0:
            neighs.append(obj)
            obj = self._merge_maze_objects(neighs)
            obj.p = min(obj.p + self.p_increse_rate,1)
        self.maze_objects.add(obj)
        obj.visulisation_publisher = self.visulisation_publisher        
        return obj

    def _merge_maze_objects(self,maze_objs):
        mean_height = np.mean([close_obj.height for close_obj in maze_objs])
        mean_pos = np.mean([close_obj.pos for close_obj in maze_objs],axis=0)
        close_images = [close_obj.image for close_obj in maze_objs]
        largest_image_idx = np.argmax([close_image.height * close_image.width for close_image in close_images])
        largest_image = close_images[largest_image_idx]
        p_max = max(obj.p for obj in maze_objs)
        classified = any(obj.classified for obj in maze_objs)
        self.maze_objects.difference_update(maze_objs)
        representative_obj = sorted(maze_objs,lambda obj_a, obj_b: obj_a.id > obj_b.id)[0]
        representative_obj.pos = mean_pos
        representative_obj.image = largest_image
        representative_obj.height = mean_height
        representative_obj.p = p_max
        representative_obj.classified = classified
        for maze_obj in maze_objs:
            maze_obj.visulisation_publisher = None
        return representative_obj
        
    def _same_color_neighbors(self,obj):
        return [maze_obj for maze_obj in self.maze_objects
                    if obj.is_close_and_same_color(maze_obj)]


class MazeObject(object):

    n_maze_objects = 0
    classified = False
    p = 0
    def __init__(self,obj_cand_msg,class_label="an_object",class_id=-1,vis_pub=None):
        
        obj_cand_point_msg = PointStamped()
        obj_cand_point_msg.header.frame_id = obj_cand_msg.header.frame_id
        obj_cand_point_msg.header.stamp = obj_cand_msg.header.stamp
        obj_cand_point_msg.point = obj_cand_msg.pos
        self.color = obj_cand_msg.color.data
        ros_sucks = True
        obj_cand_msg_new = None
        i = 0
        while ros_sucks and i <1000:
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
            self._marker.color.a = self.p
            self._vis_pub.publish(self._marker)

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
            self._marker.color.a = self.p
            self._marker.id = self.id
            self._marker.ns = "MazeObjects"
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