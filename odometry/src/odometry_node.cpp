#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>


/**
 * The main node of the odometry frame
 */

class OdometryPublisher{
public:
    ros::NodeHandle n;
    ros::Publisher odom_publisher;
    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    tf::TransformBroadcaster odom_broadcaster;




OdometryPublisher(int frequency){
    control_frequency = frequency;
    n = ros::NodeHandle("~");
    pi = 3.1416;
    pos.x = 0;
    pos.y = 0;
    pos.theta = 0;
    robot_params.base_diameter=0.26;
    robot_params.ticks_per_rotation=900;
    robot_params.wheel_radius=0.036;


    encoding_abs_prev = std::vector<int>(2,0);
    encoding_abs_new = std::vector<int>(2,0);
    encoding_delta = std::vector<double>(2,0);
    first_loop = true;

    odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 1);
    encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &OdometryPublisher::encoderCallbackLeft, this);
    encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &OdometryPublisher::encoderCallbackRight, this);



}


void encoderCallbackLeft(const phidgets::motor_encoder::ConstPtr& msg){
    encoding_abs_new[0] = msg->count;


}

void encoderCallbackRight(const phidgets::motor_encoder::ConstPtr& msg){
    encoding_abs_new[1] = -(msg->count);

}


void calculateNewPosition(){

    double dt = 1/control_frequency;

    ros::Time current_time = ros::Time::now();

    std::vector<double> dphi_dt(2, 0.0);

    if(first_loop){
        encoding_abs_prev[0] = encoding_abs_new[0];
        encoding_abs_prev[1] = encoding_abs_new[1];
    }
    first_loop =false;

    encoding_delta[0] = encoding_abs_new[0] - encoding_abs_prev[0];
    encoding_delta[1] = encoding_abs_new[1] - encoding_abs_prev[1];

    encoding_abs_prev[0] = encoding_abs_new[0];
    encoding_abs_prev[1] = encoding_abs_new[1];



    dphi_dt[0] = ((encoding_delta[0])/(robot_params.ticks_per_rotation)*2*pi)/dt;
    dphi_dt[1] = ((encoding_delta[1])/(robot_params.ticks_per_rotation)*2*pi)/dt;

    double linear_v = (robot_params.wheel_radius/2)*(dphi_dt[1] + dphi_dt[0]);
    double angular_w = (robot_params.wheel_radius/robot_params.base_diameter)*(dphi_dt[1] - dphi_dt[0]);

    double vx = linear_v*cos(pos.theta);
    double vy = linear_v*sin(pos.theta);


    pos.x += linear_v*dt*cos(pos.theta);
    pos.y += linear_v*dt*sin(pos.theta);
    pos.theta += angular_w*dt;
    if(pos.theta > pi){
        pos.theta = pos.theta-2*pi;
    }
    if(pos.theta<-pi){
        pos.theta = pos.theta+2*pi;
    }



    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos.theta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = pos.x;
    odom_trans.transform.translation.y = pos.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // Publish odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = pos.x;
    odom_msg.pose.pose.position.y = pos.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = angular_w;

    odom_publisher.publish(odom_msg);

    ROS_INFO("new Position [%f] [%f] [%f] ", pos.x, pos.y, pos.theta);

}
private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<double> encoding_delta;
    int control_frequency;
    bool first_loop;
    struct robot_parameters{
        double wheel_radius;
        double base_diameter;
        double ticks_per_rotation;
    } robot_params;
    struct robot_position{
        double x;
        double y;
        double theta;
    }pos;
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
