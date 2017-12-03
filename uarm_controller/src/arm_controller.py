#! /usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Point
from uarm.srv import MoveToJoints, MoveToJointsRequest, MoveToJointsResponse, Pump, PumpRequest, PumpResponse
from uarm_controller.srv import armPickupService, armPickupServiceRequest, armPickupServiceResponse
from math import sqrt, pow, atan2,acos,pi
import numpy as np
freq = 1
T = 1/freq
    #Calibration Error #Show error 
joint0_offset = -3.0 -5.4 -2.0
joint1_offset = -2.0 -8.0
joint2_offset = 0   - 30.0

joint0_init = 0.0
joint1_init = 141.0#30.0
joint2_init = 27.0#60.0

joint0_store = 0.0  #need to be checked
joint1_store = 141.0 #need to be checked
joint2_store = 27.0 #need to be checked

joint0_release = 0.0 #MUST be design
joint1_release = -8.0 #need to be check
joint2_release = 84.0 #need to be check

arm_movement_noise = 1

def compute_joint_angles(end_effector_goal):
    x,y,z = end_effector_goal   # in the arm frame
    xy_dist = sqrt(pow(x, 2) + pow(y, 2))
    c = sqrt(pow(4-z,2)+pow(xy_dist-8.5,2))    
    theta = atan2( (4-z),(xy_dist-8.5))
    alpha = acos( (pow(c,2)+pow(16,2)-pow(15,2))/(2*16*c) )
    beta =  acos( (pow(c,2)+pow(15,2)-pow(16,2))/(2*15*c) )

    joint0 =  atan2(y,x)*360/(2*pi)
    joint1 = (beta - theta)*360/(2*pi)
    joint2 = (alpha + theta)*360/(2*pi)
    print("Target: For x",x,"y",y,"z",z,"joint0:",joint0,"joint1:",joint1,"joint2:",joint2)
    return joint0, joint1, joint2 #ignore useless joint3 

class ArmController(object):
    
    def __init__(self):
        self.uarmService = rospy.ServiceProxy("/uarm/move_to_joints",MoveToJoints,persistent=True)
        self.pumpService = rospy.ServiceProxy("/uarm/pump",Pump,persistent=True)
        self.pump_control(False)
        self.move_to_joints(joint0_init, joint1_init, joint2_init)

    def move_to_joints(self,joint0, joint1, joint2):
        req = MoveToJointsRequest()
        req.j0 = joint0 + joint0_offset
        req.j1 = joint1 + joint1_offset
        req.j2 = joint2 + joint2_offset
        req.j3 = 0
        req.move_mode = 0
        req.movement_duration = rospy.Duration(secs=2.5)
        req.interpolation_type = 1
        req.check_limits = False
        
        resp = self.uarmService(req) #service sucess, but still need to check where it reachs
        if not resp.error:
            #print("move_to_joints: resp =",resp)
            #print("Response with offset:joint0:",resp.j0+5.4,"joint1:",resp.j1+8.0,"joint2:",resp.j2+30.0)
            #resp.elapsed
            #resp.error
            if resp.j0-req.j0+joint0_offset<1.0 and resp.j1-req.j1+joint1_offset<1.0 and resp.j2-req.j2+joint2_offset<1.0: # float equal?
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
        req.pump_status = status #bool true: open pump; false: close pump
        resp = self.pumpService(req)
        print("pump_control: resp",resp)
        return resp.pump_status
        
        
    def handle_lift(self,x,y,z):
        # From initial position to target
        print("handle_lift: input x,y,z =",x,y,z)
        #x += np.random.normal(scale=arm_movement_noise)
        #y += np.random.normal(scale=arm_movement_noise)
        #z += np.random.normal(scale=arm_movement_noise)
        #print("handle_lift: input x,y,z with noise =",x,y,z)        

        pump_result = self.pump_control(True) #bool            
        if pump_result:
            rospy.loginfo("Pump succeed")
        else:
            rospy.logerr("Pump service failed")

        self.move_arm(x,y,z+4.0)

        step1_result = self.move_arm(x,y,z-2)
        if step1_result != 0:
            if step1_result == 1:
                rospy.loginfo("Arm movement succeed")
            else:
                rospy.loginfo("Arm movement need to be adjusted")                
        else:
            rospy.logerr("Arm movement service failed")         
        

         # From target back to initial position    
        step2_result = self.move_to_joints(joint0_init, joint1_init, joint2_init)
        if step2_result !=0:
            if step2_result == 1: #1
                rospy.loginfo("Arm movement succeed")
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
        else:
            return True
            
    def handle_release(self):#def handle_release(self,pos):
        move_result = self.move_to_joints(joint0_release, joint1_release, joint2_release)
        pump_result = self.pump_control(False) #bool
        self.move_to_joints(joint0_init, joint1_init, joint2_init)
        if not pump_result:
            rospy.loginfo("Pump succeed")
        else:
            rospy.logerr("Pump service failed") 
                
        if move_result and not pump_result: 
            rospy.loginfo("Arm release succeed")           
            return True
        else:
            rospy.logerr("Arm release failed")  
            return False
            
    def handle_arm_service_request(self,request):
        point = request.pos  #unit: m; frame: arm (prefer,but base frame is OK)
        x = point.x *100.0 #unit: cm ; frame: arm
        y = point.y *100.0 #unit: cm ; frame: arm
        z = point.z *100.0 
        if request.requestType == armPickupServiceRequest.requestTypeLift:
            resp = self.handle_lift(x,y,z)  #1 for finishing
        elif request.requestType == armPickupServiceRequest.requestTypeStore:
            resp = self.handle_store()            
        elif request.requestType == armPickupServiceRequest.reqestTypeRelease:
            resp = self.handle_release()
        else:
            rospy.logerr("Invalid request type {0}".format(request.requestType))
            resp = False    
        resp_msg = armPickupServiceResponse()
        resp_msg.success = resp
        return resp_msg

def main():
    rospy.init_node("recognizer_server")
    armController = ArmController()        
    service_handle = rospy.Service("uarm/arm_controller_service",armPickupService,armController.handle_arm_service_request)
    rospy.loginfo("Everything setup")    
    rospy.spin()
    rospy.loginfo("Arm shutting down")
if __name__ == "__main__":
    main()
