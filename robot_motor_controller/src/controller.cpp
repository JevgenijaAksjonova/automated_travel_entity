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

#include <ros/ros.h>				//This header defines the classic ROS classes
#include <phidgets/motor_encoder.h> //For the message subscribed: Vreal
#include <geometry_msgs/Twist.h>	//For the message subscribed: Vref
#include <std_msgs/Float32.h>		//For the message to be published
#include <math.h>
//Declear STRUCT Var for encoder
struct motor_info
{
	float angular_velocity_left;  //left wheel real // rad/s
	float angular_velocity_right; //right wheel real// rad/s
} motor = {0.0, 0.0};

//Declear STRUCT Var for reference
struct motor_ref
{
	float angular_velocity_left;  //left wheel ref // rad/s
	float angular_velocity_right; //right wheel ref// rad/s
} reference = {0.0, 0.0};		  // -X ~ Y rad/s  200 is enough (maybe)

//Period
int32_t freq = 10;
float T = 1 / freq;

//Control Var

float error1 = 0;
float error2 = 0;
float int_error1 = 0;
float int_error2 = 0;
float kp1 = 0.0; // Need tuning //Read from YAML file
float ki1 = 0.0; // Need tuning //Read from YAML file
float kp2 = 0.0; // Need tuning //Read from YAML file
float ki2 = 0.0; // Need tuning //Read from YAML file
float pwm1 = 0;  //Output is Float32 (float 32, double 64 bits)
float pwm2 = 0;
float pwm_delta_t = 0;
ros::Time last_encoder_time_left; //For Encoder
int32_t last_count_left;
ros::Time last_encoder_time_right;
int32_t last_count_right;
ros::Time last_pwm_time;
int32_t slow_flag1 = 0, slow_flag2 = 0;
ros::Time last_slow_time1, last_slow_time2;

//Guards agains using to high pwm which will cause the system to shut down

const float PWM_MAX = 50;
const float TARGET_CUTTOFF = 5.5;
const float PWM_CUTTOFF = 3;
const float MAX_INT_ERR = 20;

float calculate_pwm(float target, float ref, float kp, float ki, float dt, float *int_error_ptr, float *error_ptr)
{
	float error = *error_ptr;
	float int_error = *int_error_ptr;
	if (std::abs(target) < TARGET_CUTTOFF)
		target = 0;
	error = target - ref;
	int_error = std::min(int_error + error * dt, MAX_INT_ERR);
	int_error = std::max(int_error + error * dt, -MAX_INT_ERR);
	float pwm = kp * error + ki * int_error;
	pwm = std::min(pwm, PWM_MAX);
	pwm = std::max(pwm, -PWM_MAX);
	if (std::abs(pwm) < PWM_CUTTOFF)
		pwm = 0;
	*int_error_ptr = int_error;
	*error_ptr = error;
	return pwm;
}

// Control function
void pwmCalc()
{

	pwm_delta_t = ros::Time::now().toSec() - last_pwm_time.toSec();
	last_pwm_time = ros::Time::now();

	pwm1 = calculate_pwm(
		reference.angular_velocity_left,
		motor.angular_velocity_left,
		kp1, ki1, pwm_delta_t,
		&int_error1, &error1);

	pwm2 = calculate_pwm(
		reference.angular_velocity_right,
		motor.angular_velocity_right,
		kp2, ki2, pwm_delta_t,
		&int_error2, &error2);
	ROS_INFO_STREAM("int_error_1 = " << int_error1 << ", int_error_2 = " << int_error2);

	//Left
	/*error1 = reference.angular_velocity_left - motor.angular_velocity_left;
	int_error1 = int_error1 + error1 * pwm_delta_t; //
	pwm1 = (kp1 * error1 + ki1 * int_error1);
	pwm1 = adjust_pwm(pwm1);
	//Right
	error2 = reference.angular_velocity_right - motor.angular_velocity_right;
	int_error2 = int_error2 + error2 * pwm_delta_t; //
	pwm2 = (kp2 * error2 + ki2 * int_error2);
	pwm2 = adjust_pwm(pwm2);*/

	ROS_INFO_STREAM("pwm1 = " << pwm1 << ", pwm2 = " << pwm2);
	/*
    //Debug
      ROS_INFO_STREAM( "left Error"<<error1<<"left Int Error:"<<(int)int_error1<<"pwm1:"<<(int)pwm1 );
      if (pwm1 > 50.0)   {pwm1 =  50.0; ROS_INFO_STREAM("PWM LIMITATION");}  //To protect the motor
      if (pwm1 < -50.0)  {pwm1 =  -50.0; ROS_INFO_STREAM("PWM LIMITATION");}   //To protect the motor
      if ((pwm1 > -8.5)&&(pwm1 < 8.5))   {last_slow_time1 =ros::Time::now(); slow_flag1=1;}  //To protect the motor
      if (slow_flag1 ==1 ){
      if ((pwm1 < -8.5)||(pwm1 > 8.5)) slow_flag1 = 0;
          else if (ros::Time::now().toSec() - last_slow_time1.toSec() >3.0) {pwm1 =  0.0; ROS_INFO_STREAM("PWM SLOW");}
      }
          //ROS_INFO_STREAM("leftPWM:"<<pwm1);

      ROS_INFO_STREAM( "Right Error"<<error2<<"Right Int Error:"<<(int)int_error2<<"pwm2:"<<(int)pwm2 );
      if (pwm2 > 50.0)   {pwm2 =  50.0; ROS_INFO_STREAM("PWM LIMITATION");}
      if (pwm2 < -50.0)  {pwm2 =  -50.0; ROS_INFO_STREAM("PWM LIMITATION");}
      if ((pwm2 > -8.5)&&(pwm2 < 8.5))   {last_slow_time2 =ros::Time::now();slow_flag2=1;}  //To protect the motor
      if (slow_flag2 ==1 ){
      if ((pwm2 < -8.5)||(pwm2 > 8.5)) slow_flag2 = 0;
          else if (ros::Time::now().toSec() - last_slow_time2.toSec() >3.0) {pwm2 =  0.0; ROS_INFO_STREAM("PWM SLOW");}
      }


*/
}

//Callback function 1: encoder left
void motorMessageReceiverLeft(const phidgets::motor_encoder &msgRecEncoderLeft)
{
	//ROS_INFO_STREAM("Left Encoder Message Receive!");

	motor.angular_velocity_left = float(msgRecEncoderLeft.count - last_count_left) * 2.0 * 3.1415 / 3591.84 / (ros::Time::now().toSec() - last_encoder_time_left.toSec()); // rad/s

	last_encoder_time_left = ros::Time::now();
	last_count_left = msgRecEncoderLeft.count;
	//ROS_INFO_STREAM("leftReal:" << motor.angular_velocity_left << "leftRef:" << reference.angular_velocity_left);
}

//Callback function 2: encoder right
void motorMessageReceiverRight(const phidgets::motor_encoder &msgRecEncoderRight)
{
	//ROS_INFO_STREAM("Right Encoder Message Receive!");
	//msgRecEncoderRight.count-last_count_right  Right Wheel Rotate in oppsite direction
	motor.angular_velocity_right = float(last_count_right - msgRecEncoderRight.count) * 2.0 * 3.1415 / 3591.84 / (ros::Time::now().toSec() - last_encoder_time_right.toSec()); // rad/s

	last_encoder_time_right = ros::Time::now();
	last_count_right = msgRecEncoderRight.count;
	//ROS_INFO_STREAM("rightReal:" << motor.angular_velocity_right << "rightRef:" << reference.angular_velocity_right);
}

//Callback function 3: reference

void refMessageReceiver(const geometry_msgs::Twist &msgRecTwist)
{

	//ROS_INFO_STREAM("Twist Message Receive!");
	reference.angular_velocity_left = float(msgRecTwist.linear.x * 2.0 - msgRecTwist.angular.z * 0.255) / (2.0 * 0.0309);  // rad/s   //Adjust robot param
	reference.angular_velocity_right = float(msgRecTwist.linear.x * 2.0 + msgRecTwist.angular.z * 0.255) / (2.0 * 0.0309); // rad/s

	//ROS_INFO_STREAM("Vref:"<<msgRec2.linear.x<<"  OmegaRef="<<msgRec2.angular.z );
	//ROS_INFO_STREAM("leftREF=" << reference.angular_velocity_left << "  rightREF=" << reference.angular_velocity_right);
	/*
	if (reference.angular_velocity_left  > 400.0)  reference.angular_velocity_left  =  400.0;   //Protect //Can be removed with analysis 
	if (reference.angular_velocity_right > 400.0)  reference.angular_velocity_right  =  400.0;

	if (reference.angular_velocity_left  < -400.0)  reference.angular_velocity_left  =  -400.0;   //Protect //Can be removed with analysis 
	if (reference.angular_velocity_right < -400.0)  reference.angular_velocity_right  =  -400.0;
*/
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "controller"); //Initialize the node, and set default name
	ros::NodeHandle nh;					 //Establish this node with Master(roscore)
	if (!nh.getParam("/motor/pid/left/kp", kp1))
	{
		ROS_ERROR("failed to detect parameter left");
		return 1;
	}
	if (!nh.getParam("/motor/pid/right/kp", kp2))
	{
		ROS_ERROR("failed to detect parameter right");
		return 1;
	}
	if (!nh.getParam("/motor/pid/left/ki", ki1))
	{
		ROS_ERROR("failed to detect parameter left");
		return 1;
	}
	if (!nh.getParam("/motor/pid/right/ki", ki2))
	{
		ROS_ERROR("failed to detect parameter right");
		return 1;
	}
	ros::Publisher pub_pwm_left = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel/left", 1);
	ros::Publisher pub_pwm_right = nh.advertise<std_msgs::Float32>("/motorcontrol/cmd_vel/right", 1);
	//Create publisher: pub_pwm	// msg_type(use :: for /) //topic_name //buffer_size
	ros::Publisher pub_dbg_error_left = nh.advertise<std_msgs::Float32>("/motorcontrol/dbg/left/error", 1);
	ros::Publisher pub_dbg_error_right = nh.advertise<std_msgs::Float32>("/motorcontrol/dbg/right/error", 1);
	ros::Publisher pub_dbg_int_left = nh.advertise<std_msgs::Float32>("/motorcontrol/dbg/left/i", 1);
	ros::Publisher pub_dbg_int_right = nh.advertise<std_msgs::Float32>("/motorcontrol/dbg/right/i", 1);
	ros::Publisher pub_dbg_ref_left = nh.advertise<std_msgs::Float32>("/motorcontrol/dbg/left/ref", 1);
	ros::Publisher pub_dbg_ref_right = nh.advertise<std_msgs::Float32>("/motorcontrol/dbg/right/ref", 1);

	ros::Subscriber sub_encoder_left = nh.subscribe("/motorcontrol/encoder/left", 1, &motorMessageReceiverLeft);
	ros::Subscriber sub_encoder_right = nh.subscribe("/motorcontrol/encoder/right", 1, &motorMessageReceiverRight);
	//ros::Subscriber sub_reference = nh.subscribe("/cmd_vel",1, &refMessageReceiver); // Rickard Teleop
	ros::Subscriber sub_reference = nh.subscribe("/motor_controller/twist", 1, &refMessageReceiver); //Enyu Teleop
	//ros::Subscriber sub_reference = nh.subscribe("/mobile_base/commands/velocity",10, &refMessageReceiver); //Testing

	ros::Rate loop_rate(freq); //f=10Hz, T=100ms

	last_encoder_time_left = ros::Time::now();
	last_count_left = 0;
	last_encoder_time_right = ros::Time::now();
	last_count_right = 0;
	last_pwm_time = ros::Time::now();

	//loop
	while (ros::ok())
	{

		std_msgs::Float32 msg_left, msg_right, msg_left_err, msg_right_err, msg_left_int, msg_right_int, msg_left_ref, msg_right_ref; //create the message

		//Calculate PWM
		pwmCalc();

		//Fill in the message
		msg_left.data = pwm1;
		msg_right.data = pwm2;
		msg_left_err.data = error1;
		msg_right_err.data = error2;
		msg_left_int.data = int_error1;
		msg_right_int.data = int_error2;
		msg_left_ref.data = reference.angular_velocity_left;
		msg_right_ref.data = reference.angular_velocity_right;

		//Publish the msg
		pub_pwm_left.publish(msg_left);
		pub_pwm_right.publish(msg_right);
		pub_dbg_error_left.publish(msg_left_err);
		pub_dbg_error_right.publish(msg_right_err);
		pub_dbg_int_left.publish(msg_left_int);
		pub_dbg_int_right.publish(msg_right_int);
		pub_dbg_ref_left.publish(msg_left_ref);
		pub_dbg_ref_right.publish(msg_right_ref);

		//Send date to rosout
		//ROS_INFO_STREAM("Sending PWM:"	<< "Left=" << msg_left.data <<"Right" << msg_right.data	);

		ros::spinOnce();

		//------------------------------------------------------------------------------------------------------
		//ROS_INFO_STREAM("leftWheel="<<motor.angular_velocity_left <<"rightWheel="<<motor.angular_velocity_right );
		//------------------------------------------------------------------------------------------------------

		loop_rate.sleep(); //sleep
	}
	//end loop
	std_msgs::Float32 stop_msg_left, stop_msg_right;
	stop_msg_left.data = 0;
	stop_msg_right.data = 0;
	pub_pwm_left.publish(stop_msg_left);
	pub_pwm_right.publish(stop_msg_right);
	ROS_INFO_STREAM("ROS stop");
	return 0;
}
