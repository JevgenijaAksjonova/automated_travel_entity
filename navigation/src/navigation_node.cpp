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
#include <nav_msgs/Odometry.h>
#include <sstream>

#include <global_path_planner.h>

using namespace std;

class Location {
  public:
    double x;
    double y;
    double theta;

    Location(double _xStart, double _yStart): xStart(_xStart), yStart(_yStart) {};
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
  private:
    double xStart;
    double yStart;
};

void Location::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  x = xStart + msg->pose.pose.position.x;
  y = yStart + msg->pose.pose.position.y;
}


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

class Path {
  public:
    double xVel;
    double yVel;
    double angVel;

    vector<pair<double,double> > globalPath;

    Path(): xVel(0), yVel(0), angVel(0), pathRad(0.05) {};
    void followPath(double x, double y);
  private:
    double pathRad;
    double distance(pair<double,double>& a, pair<double, double>& b);
};

// squared Euclidean distane
double Path::distance(pair<double, double> &a, pair<double, double> &b){
    return pow(a.first-b.first, 2) + pow(a.second-b.second, 2);
}

void Path::followPath(double x, double y) {
    pair<double, double> loc(x,y);
    while (globalPath.size() > 0 && distance(globalPath[0],loc) < pow(pathRad,2)) {
        globalPath.erase(globalPath.begin());
    }
    xVel = globalPath[0].first - loc.first;
    yVel = globalPath[0].second - loc.second;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle n;

  string mapFile = "/home/ras/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
  GlobalPathPlanner gpp(mapFile, 0.01, 0.15);
  Location loc(0.0,0.0);
  ros::Subscriber locationSub = n.subscribe("odom", 1000, &Location::callback, &loc);
  GoalPosition goal = GoalPosition();
  ros::Subscriber goalSub = n.subscribe("navigation/set_the_goal", 1000, &GoalPosition::callback, &goal);
  //ros::Subscriber subVision = n.subscribe("location", 1000, &Location::callback, &loc);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("navigation/velocity", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  Path path;
  while (ros::ok())
  {
    if (goal.changedPosition) {
        string msg = "Recalculate path";
        ROS_INFO("%s/n", msg.c_str());
        pair<double, double> startCoord(loc.x,loc.y);
        pair<double, double> goalCoord(goal.x,goal.y);
        path.globalPath = gpp.getPath(startCoord, goalCoord);
        goal.changedPosition = false;
    }

    path.followPath(loc.x,loc.y);

    geometry_msgs::Twist msg;
    msg.linear.x = path.xVel;
    msg.linear.y = path.yVel;
    msg.linear.z = 0.0;
    msg.angular.x = path.angVel;
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
