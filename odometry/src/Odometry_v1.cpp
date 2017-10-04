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
    encoder_subscriber = n.subscribe("/kobuki/encoders", 1, &StraightWalker::encoderCallback, this);


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

    std::vector<double> estimated_w(2, 0.0);

    estimated_w[0] = (encoding_delta[0])/(tick_per_rotation)*wheel_r*2*pi;
    estimated_w[1] = (encoding_delta[1])/(tick_per_rotation)*wheel_r*2*pi;

    double linear_v = ((estimated_w[0] + estimated_w[1])/2)/dt;
    double angular_v = ((estimated_w[0] - estimated_w[1])/base_d)/dt;


    current_pose.x += linear_v*(cos(current_pose.theta)*dt;
    current_pose.y += linear_v*(1/control_frequenzy)*sin(current_pose.theta)*dt;
    current_pose.theta += angular_v*(dt);


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




    ROS_INFO("new Position [%f] [%f] [%f] ", current_pose.x, current_pose.y, current_pose.theta);


    //Not finished!
    odom_msg.pose = pwr1;
    odom_msg.twist = pwr2;
    ROS_INFO("Current power [%d], [%d]\n", pwr1, pwr2);

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
