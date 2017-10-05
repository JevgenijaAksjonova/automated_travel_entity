#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class OdometryPublisher{
public:
    ros::NodeHandle n;
    ros::Publisher odom_publisher;
    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    double xpos;
    double ypos;
    double theta;





OdometryPublisher(int frequency){
    control_frequency = frequency;
    n = ros::NodeHandle("~");
    xpos = 0;
    ypos = 0;
    theta = 0;

    encoding_abs_prev = std::vector<int>(2,0);
    encoding_abs_new = std::vector<int>(2,0);
    encoding_delta = std::vector<double>(2,0);
    first_loop = 1;

    odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 1);
    encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &OdometryPublisher::encoderCallbackLeft, this);
    encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &OdometryPublisher::encoderCallbackRight, this);



}


void encoderCallbackLeft(const phidgets::motor_encoder::ConstPtr& msg){
    encoding_abs_new[0] = msg->count;


}

void encoderCallbackRight(const phidgets::motor_encoder::ConstPtr& msg){
    encoding_abs_new[1] = msg->count;

}


void calculateNewPosition(){
    double wheel_r = 0.04;
    double base_d = 0.21;
    double pi = 3.1416;
    double tick_per_rotation = 900;
    double control_frequenzy = 10; //10 hz
    double dt = 1/control_frequenzy;

    ros::Time current_time = ros::Time::now();

    std::vector<double> dphi_dt(2, 0.0);

    if(first_loop){
        encoding_abs_prev[0] = encoding_abs_new[0];
        encoding_abs_prev[1] = encoding_abs_new[1];
    }
    first_loop =0;

    encoding_delta[0] = encoding_abs_new[0] - encoding_abs_prev[0];
    encoding_delta[1] = encoding_abs_new[1] - encoding_abs_prev[1];




    dphi_dt[0] = ((encoding_delta[0])/(tick_per_rotation)*2*pi)/dt;
    dphi_dt[1] = ((encoding_delta[1])/(tick_per_rotation)*2*pi)/dt;

    double linear_v = (wheel_r/2)*(dphi_dt[0] + dphi_dt[1]);
    double angular_w = (wheel_r/base_d)*(dphi_dt[0] - dphi_dt[1]);

    double vx = linear_v*cos(theta);
    double vy = linear_v*sin(theta);


    xpos += linear_v*dt*cos(theta);
    ypos += linear_v*dt*sin(theta);
    theta += angular_w*dt;
    if(theta > 2*pi){
        theta = theta-2*pi;
    }


    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(xpos, ypos, 0.0) );
    transform.setRotation(odom_quat);
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));

    // Publish odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = xpos;
    odom_msg.pose.pose.position.y = ypos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = angular_w;

    odom_publisher.publish(odom_msg);

    ROS_INFO("new Position [%f] [%f] [%f] ", xpos, ypos, theta);

}
private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<double> encoding_delta;
    int control_frequency;
    int first_loop;
};


int main(int argc, char **argv)
{

  double frequency = 10;
  ros::init(argc, argv, "odometry_publisher");

  OdometryPublisher odom(frequency);

  ROS_INFO("Spin!");

  ros::Rate loop_rate(frequency);

  int count = 0;
  while (odom.n.ok()){

    odom.calculateNewPosition();
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
