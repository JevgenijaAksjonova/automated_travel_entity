/*
 *  uarm_controller.cpp
 *
 *
 *  Created on: Oct 11, 2017
 *  Authors:   Enyu Cao
 *            caoe <at> kth.se
 */

 //This program subscribes movement stage, and call 'move_to_joint' service ofr joint sub-destination,

 // Node Rate: 1Hz

 #include <ros/ros.h> //This header defines the classic ROS classes
 #include<geometry_msgs/Point.h>  //For the message subscribed: destination  //
 #include<uarm/MoveTo.h>       //For the service: move to node
 #include<uarm/Pump.h>       //For the service: move to node
 #include<math.h>


 //Declear STRUCT Var for subscribed message
geometry_msgs::Point start_position, target, ms1,ms2,sub_dest;   //Intial important coords


//Declear STRUCT Var for reference
int stage = 0;  // 0 for no task to move
  // 1 for y,z move ; 2 for rotation; 3 for go down; 4 for pump work;
  //To be design 5,6 Go back; 6+1 ->7 for need to put down
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

    ros::ServiceClient moveClient  = nh.serviceClient<uarm::MoveTo>("/uarm/move_to");
    ros::ServiceClient pumpClient  = nh.serviceClient<uarm::Pump>("/uarm/pump");
    //Create publisher: pub_pwm	// msg_type(use :: for /) //topic_name //buffer_size

    //Topic name: to be defined
    //Type:geometry_msgs/Point
    ros::Subscriber sub_obj = nh.subscribe("/topic",1, &objectCoordReceiver);
    //ros::Subscriber sub_reference = nh.subscribe("/mobile_base/commands/velocity",10, &refMessageReceiver); //Testing

    ros::Rate loop_rate(freq); //f=10Hz, T=100ms

    start_position.x = 0.0;
    start_position.y = 24.0;
    start_position.z = 15.0;

     uarm::MoveTo move_srv;//create the message to pub
     uarm::Pump  pump_srv;
    // Initial
            move_srv.request.position = start_position;
            move_srv.request.eef_orientation   = 0.0; // Unknow: desired orientation
            move_srv.request.move_mode         = 0; // position is absolute in the robot frame
            move_srv.request.movement_duration = ros::Duration(0.9);
            move_srv.request.ignore_orientation= false;
            move_srv.request.interpolation_type= 1; //CUBIC_INTERPOLATION = 1
            move_srv.request.check_limits      =true;

            if (moveClient.call(move_srv) ) // Response: pump_status true
                ROS_INFO("Move to inital position");
            else                            // Response: pump_status false
                ROS_ERROR("Fail to call inital MoveTo");

//loop
    while (ros::ok() ){

        uarm::MoveTo move_srv;//create the message to pub
        uarm::Pump  pump_srv;


    //calCommand();
  //-----------------------
     if (stage != 0) {
        switch (stage){
         case 1:
           sub_dest=ms1;
           break;
         case 2:
           sub_dest=ms2;
           break;
         case 3:
           sub_dest=target;
           break;
         case 4:
           // Pump
           pump_state = true;
           break;
         case 5:
           sub_dest=ms2;
           break;
         case 6:
           sub_dest=ms1;
           break;
         case 7:
           sub_dest=start_position;
           break;
         default:
           ROS_INFO_STREAM("Wrong stage "<<stage);
          }

         stage = stage + 1;
         if (stage == 8)
           stage =0; //Reach start point
  //------------------------
    if (pump_state == true){    //Fill in the message
        pump_srv.request.pump_status = true;
        if (pumpClient.call(pump_srv) ) // Response: pump_status true
            ROS_INFO("Pump on");
        else                            // Response: pump_status false
            ROS_ERROR("Fail to call Pump on");
        }
    else {
    //pump_srv.request = false; NO!!! Don't loosen it.
        move_srv.request.position = sub_dest;
        move_srv.request.eef_orientation   = 0; // Unknow: desired orientation
        move_srv.request.move_mode         = 0; // position is absolute in the robot frame
        move_srv.request.movement_duration = ros::Duration(0.9);
        move_srv.request.ignore_orientation= true;
        move_srv.request.interpolation_type= 1; //CUBIC_INTERPOLATION = 1
        move_srv.request.check_limits      =true;

        if (moveClient.call(move_srv) ) // Response: pump_status true
            ROS_INFO("Move to next destination");
        else                            // Response: pump_status false
            ROS_ERROR("Fail to call MoveTo");
    }

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
