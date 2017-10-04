#include "ros/ros.h"
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>
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
    ros::Subscriber encoder_subscriber;
    OdometryPublisher(int control_frequency); //constructor
    void calculateNewPosition();
private:
    std::vector<int> encoding_abs;
    std::vector<double> encoding_delta;
    int control_frequency;
    void encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr &msg);
    struct current_pose{
        double x;
        double y;
        double theta;
    };
};




OdometryPublisher::OdometryPublisher(int frequency){
    control_frequency = frequency;
    n = ros::NodeHandle("~");
    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.theta = 0;

    encoding_abs = std::vector<int>(2,0);
    encoding_delta = std::vector<double>(2,0);

    odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 1);
    encoder_subscriber = n.subscribe("/odom", 1, &OdometryPublisher::encoderCallback, this);


}


void OdometryPublisher::encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr& msg){
    encoding_abs[0] = msg->encoder1;
    encoding_abs[1] = msg->encoder2;

    encoding_delta[0] = (double) (msg->delta_encoder1);
    encoding_delta[1] = (double) (msg->delta_encoder2);

}


void OdometryPublisher::calculateNewPosition(){
    double wheel_r = 0.0352;
    double base_d = 0.23;
    double pi = 3.1416;
    double tick_per_rotation = 360;
    double control_frequenzy = 10; //10 hz
    double dt = 1/control_frequenzy;

    ros::Time current_time, last_time;

    std::vector<double> dphi_dt(2, 0.0);

    dphi_dt[0] = ((encoding_delta[0])/(tick_per_rotation)*2*pi)/dt;
    dphi_dt[1] = ((encoding_delta[1])/(tick_per_rotation)*2*pi)/dt;

    double linear_v = (wheel_r/2)*(dphi_dt[0] + dphi_dt[1]);
    double angular_w = (wheel_r/base_d)*(dphi_dt[0] - dphi_dt[1]);

    double vx = linear_v*(cos(current_pose.theta);
    double vy = linear_v*(sin(current_pose.theta);


    current_pose.x += linear_v*dt*(cos(current_pose.theta);
    current_pose.y += linear_v*dt*sin(current_pose.theta);
    current_pose.theta += angular_w*dt;


    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_pose.theta);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = current_pose.x;
    odom_trans.transform.translation.y = current_pose.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // Publish odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = current_pose.x;
    odom_msg.pose.pose.position.y = current_pose.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = angular_w;

    odom_publisher.publish(odom_msg);

    ROS_INFO("new Position [%f] [%f] [%f] ", current_pose.x, current_pose.y, current_pose.theta);

}


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
