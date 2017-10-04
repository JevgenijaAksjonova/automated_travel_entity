/*
 *  controller.cpp
 *
 *
 *  Created on: Oct 1, 2017
 *  Authors:   Enyu Cao
 *            caoe <at> kth.se
 */

 //This program subscribes Encoder signals, Ref Signal, and publishes PWM signal

 // Node Rate: 10Hz

 #include <ros/ros.h> //This header defines the classic ROS classes
 #include<phidgets/motor_encoder.h>  //For the message subscribed: Vreal
 #include<geometry_msgs/Twist.h>     //For the message subscribed: Vref
 #include<std_msgs/Float32.h>       //For the message to be published

 

 //Declear STRUCT Var for encoder
 struct motor_info{
	float angular_velocity_left;//left wheel real // rad/s
	float angular_velocity_right;//right wheel real// rad/s
} motor = {0.0,0.0};



//Declear STRUCT Var for reference
 struct motor_ref{
	float angular_velocity_left; //left wheel ref // rad/s
	float angular_velocity_right; //right wheel ref// rad/s
} reference = {0.0,0.0};    // -X ~ Y rad/s  200 is enough (maybe)

//Period
int32_t freq = 10;
float T = 1/freq;

//Control Var

float error1 = 0;
float error2 = 0;
float int_error1 = 0;
float int_error2 = 0;
float kp1 = 0.5;     // Need tuning
float ki1 = 0.1;      // Need tuning
float kp2 = 0.5;     // Need tuning
float ki2 = 0.1;      // Need tuning
float pwm1 = 0;   //Output is Float32 (float 32, double 64 bits)
float pwm2 = 0;

ros::Time last_encoder_time_left;        //For Encoder
int32_t   last_count_left;
ros::Time last_encoder_time_right;
int32_t   last_count_right;

// Control function
void pwmCalc(){
  
  //Left
	error1 = reference.angular_velocity_left - motor.angular_velocity_left;
	int_error1 = int_error1 + error1* T;	//T = 0.1
	pwm1 = (kp1*error1+ki1*int_error1);
	
	if (pwm1 > 30.0)  pwm1 =  30.0;  //To protect the motor
	if (pwm1 < -30.0) pwm1 = -30.0; 
	
	ROS_INFO_STREAM("leftPWM:"<<pwm1);
  //Right
	error2 = reference.angular_velocity_right - motor.angular_velocity_right;
	int_error2 = int_error2 + error2* T;	//T = 0.1
	pwm2 = (kp2*error2+ki2*int_error2);

	if (pwm2 > 30.0)  pwm2 =  30.0;
	if (pwm2 < -30.0) pwm2 = -30.0; 
		
	ROS_INFO_STREAM("leftPWM:"<<pwm2);
}



//Callback function 1: encoder left
void motorMessageReceiverLeft( const phidgets::motor_encoder & msgRecEncoderLeft){
	//ROS_INFO_STREAM("Left Encoder Message Receive!");

	motor.angular_velocity_left = float(msgRecEncoderLeft.count-last_count_left)*2.0*3.1415/48.0/(ros::Time::now().toSec()-last_encoder_time_left.toSec() );// rad/s
  
  last_encoder_time_left =ros::Time::now();
  last_count_left = msgRecEncoderLeft.count;
	ROS_INFO_STREAM("leftReal:"<<motor.angular_velocity_left);

}

//Callback function 2: encoder right
void motorMessageReceiverRight( const phidgets::motor_encoder & msgRecEncoderRight){
	//ROS_INFO_STREAM("Right Encoder Message Receive!");
//msgRecEncoderRight.count-last_count_right  Right Wheel Rotate in oppsite direction
  motor.angular_velocity_right = float(msgRecEncoderRight.count-last_count_right)*2.0*3.1415/48.0/(ros::Time::now().toSec()-last_encoder_time_right.toSec() );// rad/s
  
  last_encoder_time_right =ros::Time::now();
  last_count_right = msgRecEncoderRight.count;
	ROS_INFO_STREAM("rightReal"<<motor.angular_velocity_right);

}





//Callback function 3: reference

void refMessageReceiver( const geometry_msgs::Twist & msgRecTwist){

	ROS_INFO_STREAM("Twist Message Receive!");
	reference.angular_velocity_left = float(msgRecTwist.linear.x*2.0-msgRecTwist.angular.z*0.23)/(2.0*0.0352);// rad/s   //Adjust robot param
	reference.angular_velocity_right = float(msgRecTwist.linear.x*2.0+msgRecTwist.angular.z*0.23)/(2.0*0.0352);// rad/s

	//ROS_INFO_STREAM("Vref:"<<msgRec2.linear.x<<"  OmegaRef="<<msgRec2.angular.z );
	//ROS_INFO_STREAM("leftREF=" <<reference.angular_velocity_left <<"  rightREF=" <<reference.angular_velocity_right);

	if (reference.angular_velocity_left  > 200.0)  reference.angular_velocity_left  =  200.0;   //Protect //Can be removed with analysis 
	if (reference.angular_velocity_right  > 200.0)  reference.angular_velocity_right  =  200.0;
}





int main (int argc, char **argv){

	ros::init(argc, argv,"controller"); //Initialize the node, and set default name
	ros::NodeHandle nh;	//Establish this node with Master(roscore)

	ros::Publisher pub_pwm_left  = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel/left",1);  
	ros::Publisher pub_pwm_right = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel/right",1);  
	//Create publisher: pub_pwm	// msg_type(use :: for /) //topic_name //buffer_size

	ros::Subscriber sub_encoder_left = nh.subscribe("/motorcontrol/encoder/left",1, &motorMessageReceiverLeft);
	ros::Subscriber sub_encoder_right = nh.subscribe("/motorcontrol/encoder/right",1, &motorMessageReceiverRight);
	ros::Subscriber sub_reference = nh.subscribe("/motor_controller/twist",1, &refMessageReceiver);
	//ros::Subscriber sub_reference = nh.subscribe("/mobile_base/commands/velocity",10, &refMessageReceiver); //Testing

	ros::Rate loop_rate(freq); //f=10Hz, T=100ms

  last_encoder_time_left =ros::Time::now();
  last_count_left = 0;
  last_encoder_time_right=ros::Time::now();
  last_count_right = 0;
//loop
	while (ros::ok() ){

		std_msgs::Float32 msg_left,msg_right;//create the message

	//Calculate PWM 
		pwmCalc();

	//Fill in the message
		msg_left.data = pwm1;
		msg_right.data= pwm2;

	//Publish the msg
		pub_pwm_left.publish(msg_left);
		pub_pwm_right.publish(msg_right);

	//Send date to rosout
	//ROS_INFO_STREAM("Sending PWM:"	<< "Left=" << msg_left.data <<"Right" << msg_right.data	);

	ros::spinOnce();


//------------------------------------------------------------------------------------------------------
	//ROS_INFO_STREAM("leftWheel="<<motor.angular_velocity_left <<"rightWheel="<<motor.angular_velocity_right );
//------------------------------------------------------------------------------------------------------

	loop_rate.sleep(); //sleep 		

	}

//end loop
	ROS_INFO_STREAM("ROS stop");
	return 0;
}
