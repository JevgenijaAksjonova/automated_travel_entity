#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from uarm.srv import MoveToJoints, MoveToJointsRequest, MoveToJointsResponse, Pump, PumpRequest, PumpResponse
from uarm_controller.srv import armPickupService, armPickupServiceRequest
from math import sqrt, pow, atan2,acos,pi
freq = 1
T = 1/freq

joint0_offset = -7.0
joint2_offset = -30.0

joint0_init = 45.0
joint1_init = 30.0
joint2_init = 60.0

joint0_store = 45.0 #need to be checked
joint1_store = 30.0 #need to be checked
joint2_store = 60.0 #need to be checked

joint0_release = 45.0 #MUST be design
joint1_release = 30.0 #need to be check
joint2_release = 60.0 #need to be check

def compute_joint_angles(end_effector_goal):
    x,y,z = end_effector_goal   # in the arm frame
    xy_dist = sqrt(pow(x, 2) + pow(y, 2))
    c = sqrt(pow(4-z,2)+pow(xy_dist-5.5,2))

    theta = atan2( (4-z),(xy_dist-5.5))
    alpha = acos( (pow(c,2)+pow(16,2)-pow(15,2))/(2*16*c) )
    beta =  acos( (pow(c,2)+pow(15,2)-pow(16,2))/(2*15*c) )

    joint0 =  atan2(y,x)*360/(2*pi)
    joint1 = (beta - theta)*360/(2*pi)
    joint2 = (alpha + theta)*360/(2*pi)
    
    return joint0, joint1, joint2 #ignore useless joint3 

def ArmController(object):
    
    def __init__(self):
        self.uarmService = rospy.ServiceProxy("/uarm/move_to_joints",MoveToJoints,persistent=True)
        self.pumpService = rospy.ServiceProxy("/uarm/pump",Pump,persistent=True)
    
    def move_to_joints(self,joint0, joint1, joint2)
        req = MoveToJointsRequest()
        req.j0 = joint0 + joint0_offset
        req.j1 = joint1
        req.j2 = joint2 + joint2_offset
        req.j3 = joint3
        req.move_mode = 0
        req.movement_duration = rospy.Duration(secs=3)
        req.interpolation_type = 1
        req.check_limits = False
        
        service_success = self.uarmService(req) #service sucess, but still need to check where it reachs
        if service_success:
            resp = MoveToJointsResponse()
            #resp.elapsed
            #resp.error
            if resp.j0 == req.j0 and resp.j1 == req.j1 and resp.j2 == req.j2 # float equal?
                return 1 #movement_success
            else:                 
                return 2 #movement_fail
        else:            
            return 0 #service failed
    
    def move_arm(self,x,y,z):
        joint0, joint1, joint2 = compute_joint_angles((x,y,z))
        return self.move_to_joints(joint0, joint1, joint2)
    
    def pump_control(self,status):    
        req = PumpRequest()
        req.pump_status= status #bool true: open pump; false: close pump
        service_success = self.uarmService(req)
        if service_success:
            resp = PumpResponse()
            return resp #bool
        else
            return False
        
    def handle_lift(self,x,y,z):
        # From initial position to target
        step1_result = self.move_arm(x,y,z) : 
            if step1_result == 1:
                rospy.loginfo("Arm movement success")
            else:
                rospy.loginfo("Arm movement need to be adjusted")                
        else:
            rospy.logerr("Arm movement service failed")
         
        pump_result =  pump_control(True): #bool
            if pump_result:
                rospy.loginfo("Pump success")
            else
                rospy.logerr("Pump service failed")
         # From target back to initial position    
        step2_result =self.move_to_joints(joint0_init, joint1_init, joint2_init) :
            if step2_result == 1: #1
                rospy.loginfo("Arm movement success")
            else: #2
                rospy.loginfo("Arm movement need to be adjusted")                
        else: #0
            rospy.logerr("Arm movement service failed")            
  
        #Check result        
        if step1_result and  step2_result:
            return True
        else:
            return False
        
    def handle_store(self):        
        if not self.move_to_joints(joint0_store, joint1_store, joint2_store):
            rospy.logerr("Arm storing failed")  
            return False
        else
            retun True
            
    def handle_release(self):#def handle_release(self,pos):
        move_result = self.move_to_joints(joint0_release, joint1_release, joint2_release):
        pump_result =  pump_control(False): #bool
            if pump_result:
                rospy.loginfo("Pump success")
            else
                rospy.logerr("Pump service failed") 
                
        if move_result and pump_result: 
            rospy.loginfo("Arm release success")           
            return True
        else
            rospy.logerr("Arm release failed")  
            retun False
            
    def handle_arm_service_request(self,request):
        point = request.pos
        x,y,z = point.x,point.y,point.z
        if request.requestType == armPickupServiceRequest.requestTypeLift:
            resp = self.handle_lift(x,y,z)  #1 for finishing
        elif request.requestType == armPickupServiceRequest.requestTypeStore:
            resp = self.handle_store()            
        elif request.requestType == armPickupServiceRequest.reqestTypeRelease:
            resp = self.handle_release()
        else:
            rospy.logerr("Invalid request type {0}".format(request.requestType))
            resp = False    

def main():
    rospy.init_node("recognizer_server")
    armController = ArmController()
    
    if not self.move_to_joints(joint0_init, joint1_init, joint2_init):
        rospy.logerr("Arm initial failed")    
        
    service_handle = rospy.Service("uarm/arm_controller_service",armPickupService,armController.handle_arm_service_request)
    rospy.spin()
if __name__ == "__main__":
    main()
