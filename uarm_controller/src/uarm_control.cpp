/*
 *  uarm_controller.cpp
 *
 *  New edition: use move to joints service
 *  Created on: Oct 13, 2017
 *  Authors:   Enyu Cao
 *            caoe <at> kth.se
 */

 //This program subscribes movement stage, and call 'move_to_joint' service ofr joint sub-destination,

 // Node Rate: 1Hz

 #include <ros/ros.h> //This header defines the classic ROS classes
 #include<geometry_msgs/Point.h>  //For the message subscribed: destination  
 #include<uarm/MoveToJoints.h>       //For the service: move to joints
 #include<uarm/Pump.h>       //For the service: move to node
 #include<math.h>
// #include<uarm/Angles.h>  //For the message subscribed: destination joint  //BACK UP API
float32 j0,j1,j2,j3; //J3 is not connected.
 //Declear STRUCT Var for subscribed message
//geometry_msgs::Point start_position, target, ms1,ms2,sub_dest;   //Intial important coords


//Declear STRUCT Var for reference
bool arm_state = 0;  // 0 for no task to move
bool pump_state = 0; // Suck the object, and never loosen it.
  //To be deisgn : find a place and put it there.

//Period
int freq = 1;  // 1Hz
float T = 1.0/(float)freq;  // 1s

//Control Var

// Control function
//void calCommand(){
//}



//Callback function 1: destination message
void objectCoordReceiver( const geometry_msgs::Point & msgObjCoord){
    //ROS_INFO_STREAM("Left Encoder Message Receive!");
   float c,theta,alpha,beta;
   float y_1 = sqrt(pow(msgObjCoord.x,2)+pow(msgObjCoord.y,2); //where j0=0
   float z = msgObjCoord.z;

 
   c = sqrt(pow(4-z,2)+pow(y-5.5,2));
   theta = atan( (4-z)/(y-5.5));
   alpha = acos( (pow(c,2)+pow(16,2)-pow(15,2))/(2*16*c);
   beta =  acos( (pow(c,2)+pow(15,2)-pow(16,2))/(2*15*c);
//    ROS_INFO_STREAM("sub destinations");
   j0 = atan(msgObjCoord.x/msgObjCoord.y);//arctan
   j1 = beta - theta;
   j2 = alpha+ theta;
   j3 = 0;
}

/*Service function 1: move to node
void move_to_sub_dest( const uarm::MoveTo::Request & req,
                            uarm::MoveTo::Respond & res ){
    //ROS_INFO_STREAM("Left Encoder Message Receive!");
   double r,h;
   target = msgObjCoord;
   r = sqrt(target.x * target.x +target.y *target.y );
   if (r< 24)
    h = sqrt(12.5*12.5 -(r-11.5)*(r-11.5));
   else h = sqrt(12.5*12.5 -(36.5-r)*(36.5-r));
   ms1.x = 0; ms1.y = r; ms1.z = h;
   ms2=target; ms2.z = h;
    ROS_INFO_STREAM("sub destinations");

}
*/


int main (int argc, char **argv){

    ros::init(argc, argv,"controller"); //Initialize the node, and set default name
    ros::NodeHandle nh;	//Establish this node with Master(roscore)

    ros::ServiceClient moveClient  = nh.serviceClient<uarm::MoveToJoints>("/uarm/move_to_joints");
    ros::ServiceClient pumpClient  = nh.serviceClient<uarm::Pump>("/uarm/pump");
    //Create publisher: pub_pwm	// msg_type(use :: for /) //topic_name //buffer_size

    //Topic name: to be defined
    //Type:geometry_msgs/Point
    ros::Subscriber sub_obj = nh.subscribe("/Coord_topic",1, &objectCoordReceiver);
    //ros::Subscriber sub_reference = nh.subscribe("/mobile_base/commands/velocity",10, &refMessageReceiver); //Testing

    ros::Rate loop_rate(freq); //f=10Hz, T=100ms

     uarm::MoveToJoints move_srv;//create the message to pub
     uarm::Pump  pump_srv;
    // Initial
            move_srv.request.j0 = 45.0 -10.0; //Correction for bad 
            move_srv.request.j1 = 30.0;
            move_srv.request.j2 = 60 -30.0; //Correction for bad 
            move_srv.request.j3 = 0;
            move_srv.request.move_mode = 0; //  absolute in the robot frame
            move_srv.request.movement_duration = ros::Duration(0.9);
            move_srv.request.interpolation_type= 1; //CUBIC_INTERPOLATION = 1
            move_srv.request.check_limits      =true;

            if (moveClient.call(move_srv) ) // Response: pump_status true
                ROS_INFO("Move to inital position");
            else                            // Response: pump_status false
                ROS_ERROR("Fail to call inital MoveTo");

            pump_srv.request.pump_status = false;
        if (pumpClient.call(pump_srv) ) // Response: pump_status true
            ROS_INFO("Pump on");
        else                            // Response: pump_status false
            ROS_ERROR("Fail to call Pump on");
        
//loop
    while (ros::ok() ){

        uarm::MoveToJoint move_srv;//create the message to pub
        uarm::Pump  pump_srv;


    //calCommand();
  //-----------------------
     if (arm_state  != 0) {
        switch (arm_state ){
         case 1:           //go to target
    // 
            move_srv.request.j0 = j0 -10.0; //Correction for bad 
            move_srv.request.j1 = j1;
            move_srv.request.j2 = j2 -30.0; //Correction for bad 
            move_srv.request.j3 = j3;
            move_srv.request.move_mode = 0; //  absolute in the robot frame
            move_srv.request.movement_duration = ros::Duration(0.9);
            move_srv.request.interpolation_type= 1; //CUBIC_INTERPOLATION = 1
            move_srv.request.check_limits      =true;

            if (moveClient.call(move_srv) ) // Response: pump_status true
                ROS_INFO("Move to inital position");
            else                            // Response: pump_status false
                ROS_ERROR("Fail to call inital MoveTo");
          
           break;

         case 2:	
            pump_srv.request.pump_status = true;
        if (pumpClient.call(pump_srv) ) // Response: pump_status true
            ROS_INFO("Pump on");
        else                            // Response: pump_status false
            ROS_ERROR("Fail to call Pump on");
        }
           break;
         case 3:
            move_srv.request.j0 = 45.0 -10.0; //Correction for bad 
            move_srv.request.j1 = 30.0;
            move_srv.request.j2 = 60 -30.0; //Correction for bad 
            move_srv.request.j3 = 0;
            move_srv.request.move_mode = 0; //  absolute in the robot frame
            move_srv.request.movement_duration = ros::Duration(0.9);
            move_srv.request.interpolation_type= 1; //CUBIC_INTERPOLATION = 1
            move_srv.request.check_limits      =true;

            if (moveClient.call(move_srv) ) // Response: pump_status true
                ROS_INFO("Move to inital position");
            else                            // Response: pump_status false
                ROS_ERROR("Fail to MoveTo");
         break;
         default:
           ROS_INFO_STREAM("Wrong stage "<<stage);
          }

         stage = stage + 1;
         if (arm_state  == 4)
           arm_state  =0; //Reach start point
  //------------------------
   
    //Send date to rosout
    //ROS_INFO_STREAM("Sending PWM:"	<< "Left=" << msg_left.data <<"Right" << msg_right.data	);
}
    ros::spinOnce();


//------------------------------------------------------------------------------------------------------
    //ROS_INFO_STREAM("="<<var );
//------------------------------------------------------------------------------------------------------

    loop_rate.sleep(); //sleep

    }

//end loop
    ROS_INFO_STREAM("ROS stop");
    return 0;
}
