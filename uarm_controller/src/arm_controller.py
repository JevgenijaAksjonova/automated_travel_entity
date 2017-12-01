#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from uarm.srv import MoveToJoints, MoveToJointsRequest, PumpRequest, Pump
from uarm_controller.srv import armPickupService, armPickupServiceRequest
from math import sqrt, pow, atan2,acos,pi
freq = 1
T = 1/freq

joint0_offset = -7.0
joint2_offset = -30.0

def compute_joint_angles(end_effector_goal):
    x,y,z = end_effector_goal
    xy_dist = sqrt(pow(x, 2) + pow(y, 2))
    c = sqrt(pow(4-z,2)+pow(xy_dist-5.5,2))

    theta = atan2( (4-z),(xy_dist-5.5))
    alpha = acos( (pow(c,2)+pow(16,2)-pow(15,2))/(2*16*c) )
    beta =  acos( (pow(c,2)+pow(15,2)-pow(16,2))/(2*15*c) )

    joint0 =  atan2(y,x)*360/(2*pi)
    joint1 = (beta - theta)*360/(2*pi)
    joint2 = (alpha + theta)*360/(2*pi)
    
    return joint0, joint1, joint2

def ArmController(object):
    
    def __init__(self):
        self.uarmService = rospy.ServiceProxy("/uarm/move_to_joints",MoveToJoints,persistent=True)
        self.pumpService = rospy.ServiceProxy("/uarm/pump",Pump,persistent=True)
    
    def move_arm(self,x,y,z):
        joint0, joint1, joint2 = compute_joint_angles((x,y,z))
        req = MoveToJointsRequest()
        req.j0 = joint0 + joint0_offset
        req.j1 = joint1
        req.j2 = joint2 + joint2_offset
        req.j3 = joint3
        req.move_mode = 0
        req.movement_duration = rospy.Duration(secs=3)
        req.interpolation_type = 1
        req.check_limits = False
        movement_success = self.uarmService(req)
        return movement_success

    def handle_lift(x,y,z):
       
        if self.move_arm(x,y,z):
            rospy.loginfo("Arm movement success")
        else:
            rospy.logerr("Arm movement failed")

    def handle_release(pos):
        raise NotImplementedError()

    def handle_store():
        raise NotImplementedError()


    def handle_arm_service_request(request):
        point = request.pos
        x,y,z = point.x,point.y,point.z
        if request.requestType == armPickupServiceRequest.requestTypeLift:
            resp = handle_lift(x,y,z)
        elif request.requestType == armPickupServiceRequest.reqestTypeRelease:
            resp = handle_release(x,y,z)
        elif request.requestType == armPickupServiceRequest.requestTypeStore:
            resp = handle_store()
        else:
            rospy.logerr("Invalid request type {0}".format(request.requestType))
            resp = False
    

    

def main():
    rospy.init_node("recognizer_server")
    armController = ArmController()
    
    def test_handle_arm_service_request(msg):
        req = armPickupServiceRequest()
        req.pos = msg
        req.requestType = armPickupServiceRequest.requestTypeLift
        armController.handle_arm_service_request(req)
    rospy.SubscribeListener("uarm/debug/test_handle_arm_service_request",Point,test_handle_arm_service_request,queue_size=1)
    service_handle = rospy.Service("uarm/arm_controller_service",armPickupService,armController.handle_arm_service_request)
    rospy.spin()
if __name__ == "__main__":
    main()