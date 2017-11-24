#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <std_msgs/Bool.h>



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;

class OdometryPublisher{
public:
    ros::NodeHandle n;
    ros::Publisher odom_publisher;
    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    ros::Subscriber filter_subscriber;
    ros::Subscriber position_update_subscriber;

    double xpos;
    double ypos;
    double theta;
    double pi;
    double _filterX;
    double _filterY;
    double _filterTheta;
    bool _update_position;


OdometryPublisher(int frequency){
    control_frequency = frequency;
    n = ros::NodeHandle("~");
    pi = 3.1416;
    xpos = 0.215;
    ypos = 0.230;
    theta = pi/2;
    _update_position = false;
    _strikes = 0;

    if(!n.getParam("/odom/MAXIMUM_DEVIATION_ALLOWED",MAXIMUM_DEVIATION_ALLOWED)){
        ROS_ERROR("odom failed to detect parameter 1");
        exit(EXIT_FAILURE);
    }
    if(!n.getParam("/odom/MAXIMUM_NUMBER_OF_STRIKES_ALLOWED",MAXIMUM_NUMBER_OF_STRIKES_ALLOWED)){
        ROS_ERROR("odom failed to detect parameter 2");
        exit(EXIT_FAILURE);
    }

    ROS_INFO("ODOM running with params:");
    ROS_INFO("Deviation allowed: %f", MAXIMUM_DEVIATION_ALLOWED);
    ROS_INFO("Strikes allowed : %d", MAXIMUM_NUMBER_OF_STRIKES_ALLOWED);



    encoding_abs_prev = std::vector<int>(2,0);
    encoding_abs_new = std::vector<int>(2,0);
    encoding_delta = std::vector<double>(2,0);
    first_loop = 1;

    odom_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
    encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &OdometryPublisher::encoderCallbackLeft, this);
    encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &OdometryPublisher::encoderCallbackRight, this);
    filter_subscriber = n.subscribe("/filter", 1, &OdometryPublisher::filterCallback, this);
    position_update_subscriber = n.subscribe("/odom/update", 1, &OdometryPublisher::positionUpdateCallback, this);


}


void encoderCallbackLeft(const phidgets::motor_encoder::ConstPtr& msg){
    encoding_abs_new[0] = msg->count;


}

void encoderCallbackRight(const phidgets::motor_encoder::ConstPtr& msg){
    encoding_abs_new[1] = -(msg->count);

}

void filterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  _filterX = msg->pose.pose.position.x;//xStart - msg->pose.pose.position.y;
  _filterY = msg->pose.pose.position.y;//yStart + msg->pose.pose.position.x;

  geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
  _filterTheta = tf::getYaw(odom_quat);
  monitorPositionDeviation();


}

void positionUpdateCallback(const std_msgs::Bool::ConstPtr& update)
{
  _update_position = update->data;
}

void updatePositionAccordingToFilter(){
    ROS_INFO("Updating odom position!");
    xpos = _filterX;
    ypos = _filterY;
    theta = _filterTheta;

    _update_position = false;
}

void monitorPositionDeviation(){
    float deviation =  sqrt(pow((_filterX - xpos),2) + pow((_filterY - ypos), 2));
    //ROS_INFO("odom says: x = %f y = %f theta = %f", xpos, ypos, theta);
    //ROS_INFO("filter says x = %f y = %f theta = %f", _filterX, _filterY, _filterTheta);
    if(deviation > MAXIMUM_DEVIATION_ALLOWED){
        _strikes ++;
    }else{
        if(_strikes >0){
            _strikes --;
        }
    }
    // ROS_INFO("Deviation = [%f] strikes = [%d]", deviation, _strikes);

    if(_strikes >= MAXIMUM_NUMBER_OF_STRIKES_ALLOWED){
        _update_position = true;
        _strikes = 0;
    }
    
}

void calculateNewPosition(){
    double wheel_r = 0.037;
    double base_d = 0.21;
    double tick_per_rotation = 900;
    double control_frequenzy = 100; //10 hz
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

    encoding_abs_prev[0] = encoding_abs_new[0];
    encoding_abs_prev[1] = encoding_abs_new[1];



    dphi_dt[0] = ((encoding_delta[0])/(tick_per_rotation)*2*pi)/dt;
    dphi_dt[1] = ((encoding_delta[1])/(tick_per_rotation)*2*pi)/dt;

    double linear_v = (wheel_r/2)*(dphi_dt[1] + dphi_dt[0]);
    double angular_w = (wheel_r/base_d)*(dphi_dt[1] - dphi_dt[0]);

    double vx = linear_v*cos(theta);
    double vy = linear_v*sin(theta);


    xpos += linear_v*dt*cos(theta);
    ypos += linear_v*dt*sin(theta);
    theta += angular_w*dt;
    if(theta > pi){
        theta = theta-2*pi;
    }
    if(theta<-pi){
        theta = theta+2*pi;
    }


    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //odom_broadcaster.sendTransform(odom_trans);

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

}
private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<double> encoding_delta;
    int control_frequency;
    int first_loop;
    float MAXIMUM_DEVIATION_ALLOWED;
    int _strikes;
    int MAXIMUM_NUMBER_OF_STRIKES_ALLOWED;
};


int main(int argc, char **argv)
{

  double frequency = 100;
  ros::init(argc, argv, "odometry_publisher");

  OdometryPublisher odom(frequency);

  ROS_INFO("Spin!");

  ros::Rate loop_rate(frequency);

  int count = 0;
  while (odom.n.ok()){
      if(odom._update_position){
          odom.updatePositionAccordingToFilter();
      }

    odom.calculateNewPosition();
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

