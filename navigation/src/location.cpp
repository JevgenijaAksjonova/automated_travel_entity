#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <location.h>

using namespace std;

Location::Location(double _xStart, double _yStart, double _thetaStart):
    xStart(_xStart),
    yStart(_yStart),
    thetaStart(_thetaStart) {
    x=xStart;
    y=yStart;
    theta = thetaStart;
}

void Location::callback(const nav_msgs::Odometry::ConstPtr& msg)
{

  x = xStart - msg->pose.pose.position.y;
  y = yStart + msg->pose.pose.position.x;

  geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
  theta = thetaStart + tf::getYaw(odom_quat);

  stringstream s;
  s << "Received position: " << x << " " << y << " "<< theta;
  //ROS_INFO("%s/n", s.str().c_str());
}

