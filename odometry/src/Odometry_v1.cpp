#include "ros/ros.h"
#include <ras_lab1_msgs/PWM.h>
#include <ras_lab1_msgs/Encoders.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class Odometry_v1{
public:
    ros::NodeHandle n;
    ros::Publisher odom_publisher;
    ros::Subscriber twist_subscriber;
    ros::Subscriber encoder_subscriber;


struct current_pose{
    double x;
    double y;
    double theta;
};

Odometry_v1(){

    n = ros::NodeHandle("~");
    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.theta = 0;

    encoding_abs = std::vector<int>(2,0);
    encoding_delta = std::vector<double>(2,0);

    odom_publisher = n.advertise<nav_msgs::Odometry>("/kobuki/odom_v1", 1);
    encoder_subscriber = n.subscribe("/kobuki/encoders", 1, &StraightWalker::encoderCallback, this);


}


void encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr& msg){
    encoding_abs[0] = msg->encoder1;
    encoding_abs[1] = msg->encoder2;

    encoding_delta[0] = (double) (msg->delta_encoder1);
    encoding_delta[1] = (double) (msg->delta_encoder2);

//    ROS_INFO("Absolute encoding values [%d], [%d]\n", encoding_abs[0], encoding_abs[1]);
//    ROS_INFO("delta encoding values [%d], [%d]\n", encoding_delta[0], encoding_delta[1]);

}


void calculateNewPosition(){
    double wheel_r = 0.0352;
    double base_d = 0.23;
    double pi = 3.1416;
    double tick_per_rotation = 360;
    double control_frequenzy = 10; //10 hz
    nav_msgs::Odometry ;

    //convert to radians
    std::vector<double> estimated_w(2, 0.0);

    estimated_w[0] = (encoding_delta[0]*2*pi*control_frequenzy)/(tick_per_rotation);
    estimated_w[1] = (encoding_delta[1]*2*pi*control_frequenzy)/(tick_per_rotation);

    double linear_v = ((estimated_w[0] + estimated_w[0])/2)*wheel_r;
    double angular_v = (estimated_w[0] - estimated_w[0])*wheel_r/base_d;

    current_pose.x = current_pose.x + linear_v*(1/control_frequenzy)*cos(current_pose.theta);
    current_pose.y = current_pose.y + linear_v*(1/control_frequenzy)*sin(current_pose.theta);
    current_pose.theta = current_pose.theta + angular_v*(1/control_frequenzy);


    ROS_INFO("new Position [%f] [%f] [%f] ", current_pose.x, current_pose.y, current_pose.theta);


    //Not finished!
    odom_msg.pose = pwr1;
    odom_msg.twist = pwr2;
    ROS_INFO("Current power [%d], [%d]\n", pwr1, pwr2);

}
private:

    std::vector<int> encoding_abs;
    std::vector<double> encoding_delta;


};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Odometry_v1");

  Odometry_v1 odom;

  ROS_INFO("Spin!");

  ros::Rate loop_rate(10);

  int count = 0;
  while (odom.n.ok()){

    odom.calculateNewPosition();
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
