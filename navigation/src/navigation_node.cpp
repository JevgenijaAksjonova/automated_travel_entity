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
#include <math.h>

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

    GoalPosition(): x(0), y(0), theta(0), changedPosition(false), changedAngle(false) {};
    void callback(const geometry_msgs::Twist::ConstPtr& msg);
};

void GoalPosition::callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double x_new = msg->linear.x;
  double y_new = msg->linear.y;
  double theta_new = msg->angular.x;

  if (x_new != x || y_new != y || theta_new != theta}) {
      x = x_new;
      y = y_new;
      theta = theta_new;
      stringstream s;
      s << "New goal position: " << x << " " << y << " " << theta;
      ROS_INFO("%s/n", s.str().c_str());
      changedPosition = true;
  }

}

class Path {
  public:
    double linVel;
    double angVel;
    bool move;

    vector<pair<double,double> > globalPath;

    Path(): linVel(0), angVel(0), pathRad(0.05), distanceTol(0.01), move(false) {};
    setGoal(double x, double y, double theta);
    void followPath(double x, double y, double theta);
  private:
    double pathRad;
    double distanceTol;
    double goalX;
    double goalY;
    double goalAng;
    double distance(pair<double,double>& a, pair<double, double>& b);
};

void setGoal(double x, double y, double theta) {
    goalX = x;
    goalY = y;
    goalAng = theta;
}

// squared Euclidean distane
double Path::distance(pair<double, double> &a, pair<double, double> &b){
    return pow(a.first-b.first, 2) + pow(a.second-b.second, 2);
}

void Path::followPath(double x, double y) {
    pair<double, double> loc(x,y);
    if (globalPath.size() > 0 ) {
        while (globalPath.size() > 0 && distance(globalPath[0],loc) < pow(pathRad,2)) {
            globalPath.erase(globalPath.begin());
        }
        linVel = sqrt(globalPath[0], loc);
        angVel = //globalPath[0].second - loc.second;
    } else if (distance(pair<double,double>(goalX,goalY),loc) < pow(distanceTol,2)) {
        linVel = sqrt(pair<double,double>(goalX,goalY), loc);
        angVel = //globalPath[0].second - loc.second;
    } else if ( ) {
        linVel = 0;
        angVel = ;
    } else {
        string msg = "Goal is reached!";
        ROS_INFO("%s/n", msg.c_str());
        move = false;
        linVel = 0;
        angVel = 0;
    }
}

void Path::obstaclesCallback() {

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
  Path path;
  ros::Subscriber subObstacles = n.subscribe("navigation/obstacles", 1000, &Path::obstaclesCallback, &path);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("navigation/velocity", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    if (goal.changedPosition) {
        string msg = "Recalculate path";
        ROS_INFO("%s/n", msg.c_str());
        pair<double, double> startCoord(loc.x,loc.y);
        pair<double, double> goalCoord(goal.x,goal.y);
        path.globalPath = gpp.getPath(startCoord, goalCoord);
        path.setGoal(goal.x, goal.y, goal.theta);
        goal.changedPosition = false;
        path.move = true;
    }

    if (path.move) {
        path.followPath(loc.x,loc.y,loc.theta);
    } else {
      path.linVel = 0;
      path.angVel = 0;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = path.xVel;
    msg.linear.y = 0.0;
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
