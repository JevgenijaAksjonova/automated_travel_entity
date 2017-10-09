/*
 *  navigation_node.cpp
 *
 *
 *  Created on: Oct 8, 2017
 *  Authors:   Jevgenija Aksjonova
 *            jevaks <at> kth.se
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

#include <global_path_planner.h>

using namespace std;
/*
class Location {
  public:
    int d1 = 0;
    int d2 = 0;
    void callback(const ras_lab1_msgs::ADConverter::ConstPtr& msg);
};

void Location::callback(const ras_lab1_msgs::ADConverter::ConstPtr& msg)
{
  d1 = msg->ch1;
  d2 = msg->ch2;
}
*/

class GoalPosition {
  public:
    double x;
    double y;
    double theta;
    bool changedPosition;
    bool changedAngle;

    GoalPosition(): x(0), y(0), theta(0), changedPosition(false), changedAngle(false) {};
    void callback(const geometry_msgs::Twist::ConstPtr& msg);
};

void GoalPosition::callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double x_new = msg->linear.x;
  double y_new = msg->linear.y;
  double theta_new = msg->angular.x;

  if (x_new != x || y_new != y ) {
      x = x_new;
      y = y_new;
      theta = theta_new;
      stringstream s;
      s << "New goal position: " << x << " " << y << " " << theta;
      ROS_INFO("%s/n", s.str().c_str());
      changedPosition = true;
  }
  if (theta_new != theta) {
      theta = theta_new;
      changedAngle = true;
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle n;

  //Location loc;
  //ros::Subscriber locationSub = n.subscribe("location", 1000, &Location::callback, &loc);
  GoalPosition goal = GoalPosition();
  ros::Subscriber goalSub = n.subscribe("navigation/set_the_goal", 1000, &GoalPosition::callback, &goal);
  //ros::Subscriber subVision = n.subscribe("location", 1000, &Location::callback, &loc);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("navigation/velocity", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    if (goal.changedPosition) {
        string msg = "Recalculate path";
        ROS_INFO("%s/n", msg.c_str());
    }

    geometry_msgs::Twist msg;
    msg.linear.x = goal.x;
    msg.linear.y = goal.y;
    msg.linear.z = 0.0;
    msg.angular.x = goal.theta;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    //ROS_INFO("%s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
