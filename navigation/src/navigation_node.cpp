/*
 *  navigation_node.cpp
 *
 *
 *  Created on: Oct 8, 2017
 *  Authors:   Jevgenija Aksjonova
 *            jevaks <at> kth.se
 */

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <pwd.h>
#include <memory>

#include <location.h>
#include <path.h>
#include <global_path_planner.h>
#include <map_visualization.h>
#include <project_msgs/stop.h>
#include "project_msgs/direction.h"
#include "project_msgs/global_path.h"

using namespace std;

class GoalPosition {
  public:
    double x;
    double y;
    double theta;
    bool changedPosition;
    bool path_found;

    GoalPosition(shared_ptr<GlobalPathPlanner> _gpp, shared_ptr<Location> _loc, shared_ptr<Path> _path);
    void callback(double x_new, double y_new, double theta_new);
    void publisherCallback(const geometry_msgs::Twist::ConstPtr& msg);
    bool serviceCallback(project_msgs::global_path::Request &request,
                         project_msgs::global_path::Response &response);
  private:
    shared_ptr<GlobalPathPlanner> gpp;
    shared_ptr<Location> loc;
    shared_ptr<Path> path;
};

GoalPosition::GoalPosition(shared_ptr<GlobalPathPlanner> _gpp, shared_ptr<Location> _loc, shared_ptr<Path> _path):
             x(0), y(0), theta(0), gpp(_gpp), loc(_loc), path(_path),
             changedPosition(false) {
}

void GoalPosition::publisherCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double x_new = msg->linear.x;
  double y_new = msg->linear.y;
  double theta_new = msg->angular.x;

  callback(x_new,y_new,theta_new);

}

bool GoalPosition::serviceCallback(project_msgs::global_path::Request &request,
                                   project_msgs::global_path::Response &response)
{
  double x_new = request.pose.linear.x;
  double y_new = request.pose.linear.y;
  double theta_new = request.pose.angular.x;

  callback(x_new, y_new, theta_new);
  response.path_found = path_found;

  return true;

}

void GoalPosition::callback(double x_new, double y_new, double theta_new) {

    stringstream s;
    s << "Received the goal position: " << x_new << " " << y_new << " " << theta_new;
    ROS_INFO("%s/n", s.str().c_str());

    if ((x_new != x)||(y_new != y)||(theta_new != theta)) {
        x = x_new;
        y = y_new;
        theta = theta_new;
        stringstream s;
        s << "New goal position: " << x << " " << y << " " << theta;
        ROS_INFO("%s/n", s.str().c_str());
        changedPosition = true;
    }

    if (changedPosition) {
        string msg = "Recalculate path";
        ROS_INFO("%s/n", msg.c_str());
        pair<double, double> startCoord(loc->x,loc->y);
        pair<double, double> goalCoord(x,y);
        vector<pair<double,double> >  globalPath = gpp->getPath(startCoord, goalCoord);
        if (globalPath.size() == 0) {
            stringstream s;
            s << "Cant find a global path! Location " << loc->x <<" "<< loc->y;
            ROS_INFO("%s/n", s.str().c_str());
            path_found = false;
        } else {
            stringstream s;
            s << "Path is found, size" << globalPath.size();
            ROS_INFO("%s/n", s.str().c_str());
            path->setPath(x, y, theta, globalPath);
            changedPosition = false;
            path_found = true;
        }
    }
}

string getHomeDir() {
    passwd* pw = getpwuid(getuid());
    string path(pw->pw_dir);
    return path;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle n;

  // Global Path Planner
  string mapFile = getHomeDir()+"/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
  double gridCellSize = 0.01;
  double robotRadius = 0.17;
  shared_ptr<GlobalPathPlanner> gpp = make_shared<GlobalPathPlanner>(mapFile, gridCellSize, robotRadius);

  MapVisualization mapViz(gpp);
  stringstream s;
  s << "Grid Size " << gpp->gridSize.first <<" "<< gpp->gridSize.second << " Scale "<< gpp->mapScale.first << " cell "<< gpp->cellSize;
  ROS_INFO("%s/n", s.str().c_str());

  // Location
  shared_ptr<Location> loc = make_shared<Location>(0.215,0.224, M_PI/2.0);
  ros::Subscriber locationSub = n.subscribe("/odom", 1, &Location::callback, loc.get());

  // Path
  double pathRad = 0.25;
  double distanceTol = 0.05;
  double angleTol = 2*M_PI;
  shared_ptr<Path> path = make_shared<Path>(pathRad, distanceTol, angleTol);
  path->lppService = n.serviceClient<project_msgs::direction>("local_path");
  path->statusPub = n.advertise<std_msgs::Bool>("navigation/status", 1);
  // emergency stop
  ros::Subscriber subObstacles = n.subscribe("navigation/obstacles", 1000, &Path::obstaclesCallback, path.get());

  // Goal
  GoalPosition goal = GoalPosition(gpp, loc, path);
  ros::Subscriber goalSub = n.subscribe("navigation/set_the_goal_test", 1, &GoalPosition::publisherCallback, &goal);
  ros::ServiceServer service = n.advertiseService("navigation/set_the_goal", &GoalPosition::serviceCallback, &goal);


  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1);
  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    if (path->move) {
        path->followPath(loc->x,loc->y,loc->theta);
        stringstream s;
        s << "Follow path " << path->linVel << " " << path->angVel;
        ROS_INFO("%s/n", s.str().c_str());
    } else {
      path->linVel = 0;
      path->angVel = 0;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = path->linVel;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = path->angVel;

    //ROS_INFO("%s", msg.data.c_str());
    if (path->move) {
        //pub.publish(msg);
    }

    mapViz.publishMap();
    mapViz.publishPath(path->globalPath);
    mapViz.publishDirection(path->linVel,path->angVel);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}


