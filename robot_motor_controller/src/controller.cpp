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

 //------------------Need Change-------------------------------

//------------------Need Change-------------------------------

 #include<phidgets/motor_encoder.h>  //For the message subscribed: Vreal

 #include<geometry_msgs/Twist.h>     //For the message subscribed: Vref

 #include<std_msgs/Float32.h>       //For the message to be published

 //------------------Need Change-------------------------------

 //------------------Need Change-------------------------------

 

 //Declear STRUCT Var for encoder

 struct motor_info{

	float angular_velocity_left;//left wheel real // rad/s

	float angular_velocity_right;//right wheel real// rad/s

} motor = {0,0};



//Declear STRUCT Var for reference

 struct motor_ref{

	float angular_velocity_left; //left wheel ref // rad/s

	float angular_velocity_right; //right wheel ref// rad/s

} reference = {0.0,0.0};    // -X ~ Y rad/s 

//Period

int32_t T = 0.1;

//Control Var

float error1 = 0;

float error2 = 0;

float int_error1 = 0;

float int_error2 = 0;

float kp1 = 10;     // Need tuning

float ki1 = 2;      // Need tuning

float kp2 = 10;     // Need tuning

float ki2 = 2;      // Need tuning

float pwm1 = 0;   //Output is Float32 (float 32, double 64 bits)

float pwm2 = 0;

// Control function

void pwmCalc(){

	error1 = reference.angular_velocity_left - motor.angular_velocity_left;

	int_error1 = int_error1 + error1* T;	//T = 0.1

	pwm1 = (kp1*error1+ki1*int_error1);



	error2 = reference.angular_velocity_right - motor.angular_velocity_right;

	int_error2 = int_error2 + error2* T;	//T = 0.1

	pwm2 = (kp2*error2+ki2*int_error2);



	if (pwm1 > 30.0)  pwm1 =  30.0;

	if (pwm1 < -30.0) pwm1 = -300.0; 

	if (pwm2 > 30.0)  pwm2 =  30.0;

	if (pwm2 < -30.0) pwm2 = -30.0; 

	

}



//Callback function 1: encoder left

void motorMessageReceiverLeft( const phidgets::motor_encoder & msgRecEncoderLeft){



	ROS_INFO_STREAM("Left Encoder Message Receive!");

	motor.angular_velocity_left = 10*float(msgRecEncoderLeft.count_change)*2.0*3.1415/360.0;// rad/s

	//ROS_INFO_STREAM("leftReal:"<<motor.angular_velocity_left<<"  rightReal"<<motor.angular_velocity_right);

}

//Callback function 1: encoder right

void motorMessageReceiverRight( const phidgets::motor_encoder & msgRecEncoderRight){



	ROS_INFO_STREAM("Right Encoder Message Receive!");

  motor.angular_velocity_right = 10*float(msgRecEncoderRight.count_change)*2.0*3.1415/360.0;// rad/s

	//ROS_INFO_STREAM("leftReal:"<<motor.angular_velocity_left<<"  rightReal"<<motor.angular_velocity_right);

}





//Callback function 2: reference

void refMessageReceiver( const geometry_msgs::Twist & msgRecTwist){



	ROS_INFO_STREAM("Twist Message Receive!");

	reference.angular_velocity_left = float(msgRecTwist.linear.x*2.0-msgRecTwist.angular.z*0.23)/(2.0*0.0352);// rad/s   //Adjust robot param

	reference.angular_velocity_right = float(msgRecTwist.linear.x*2.0+msgRecTwist.angular.z*0.23)/(2.0*0.0352);// rad/s

	

	//ROS_INFO_STREAM("Vref:"<<msgRec2.linear.x<<"  OmegaRef="<<msgRec2.angular.z );

	//ROS_INFO_STREAM("leftREF=" <<reference.angular_velocity_left <<"  rightREF=" <<reference.angular_velocity_right);

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



	ros::Rate loop_rate(10); //f=10Hz, T=100ms



	uint32_t count = 0; // For header.seq



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

	//ROS_INFO_STREAM("Sending PWM:"	<< "Left=" << msg.PWM1 <<"Right" << msg.PWM2	);

	

	ros::spinOnce();

	count++;



//------------------------------------------------------------------------------------------------------

	ROS_INFO_STREAM("leftWheel="<<motor.angular_velocity_left <<"rightWheel="<<motor.angular_velocity_right );

//------------------------------------------------------------------------------------------------------



	loop_rate.sleep(); //sleep 		

	}

//end loop

	ROS_INFO_STREAM("ROS stop");

	return 0;



}
