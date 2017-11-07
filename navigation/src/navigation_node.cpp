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

#include <global_path_planner.h>
#include <map_visualization.h>
#include <project_msgs/stop.h>
#include "project_msgs/direction.h"
#include "project_msgs/global_path.h"

using namespace std;

class Location {
  public:
    double x;
    double y;
    double theta;

    Location(double _xStart, double _yStart, double _thetaStart):
        xStart(_xStart),
        yStart(_yStart),
        thetaStart(_thetaStart) {
        x=xStart;
        y=yStart;
        theta = thetaStart;
    };
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
  private:
    double xStart;
    double yStart;
    double thetaStart;
};

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


class Path {
  public:
    double linVel;
    double angVel;
    bool move;

    vector<pair<double,double> > globalPath;

    ros::ServiceClient lppService;
    ros::Publisher statusPub;

    Path(): linVel(0), angVel(0), pathRad(0.20), distanceTol(0.05), angleTol(2*M_PI), move(false) {};
    void setGoal(double x, double y, double theta);
    void followPath(double x, double y, double theta);
    void obstaclesCallback(const project_msgs::stop::ConstPtr& msg);
    void setPath(double x, double y, double theta, vector<pair<double,double> > path);
  private:
    double pathRad;
    double distanceTol;
    double angleTol;
    double goalX;
    double goalY;
    double goalAng;
    double distance(pair<double,double>& a, pair<double, double>& b);
    double getAngle(pair<double,double> &g, pair<double, double> &p);
    double diffAngles(double a, double b) ;
    void amendDirection();
};

void Path::setPath(double x, double y, double theta, vector<pair<double,double> > path) {
    globalPath = path;
    setGoal(x, y, theta);
    move = true;
}

void Path::setGoal(double x, double y, double theta) {
    goalX = x;
    goalY = y;
    goalAng = theta;
}

// Euclidean distane
double Path::distance(pair<double, double> &a, pair<double, double> &b){
    return sqrt (pow(a.first-b.first, 2) + pow(a.second-b.second, 2) );
}

// Returns angle in the interval (-Pi;Pi]
double Path::getAngle(pair<double,double> &g, pair<double, double> &p) {
    double x = g.first - p.first;
    double y = g.second - p.second;
    if (x == 0) {
        if (y >= 0) {
            return M_PI/2.0;
        } else {
            return -M_PI/2.0;
        }
    }
    double angle = atan(y/x);
    if (x > 0) {
        return angle;
    } else {
        if (y >= 0) {
            return angle + M_PI;
        } else {
            return angle - M_PI;
        }
    }
    // return atan2(y,x);
}

double Path::diffAngles(double a, double b) {
    double diff = a-b;
    while (diff > M_PI) {
        diff -= 2*M_PI ;
    }
    while (diff <= - M_PI) {
        diff += 2*M_PI;
    }
    return diff;
}

void Path::followPath(double x, double y, double theta) {
    pair<double, double> loc(x,y);
    pair<double,double> goal(goalX,goalY);
    double dist = distance(goal,loc);
    if (globalPath.size() > 0 ) {
        while (globalPath.size() > 1 && 
                   (distance(globalPath[0],loc) < pathRad ||
                    distance(globalPath[1], loc) < distance(globalPath[0], loc))
               ) {
            globalPath.erase(globalPath.begin());
        }
        linVel = distance(globalPath[0], loc);
        double targetAng = getAngle(globalPath[0],loc);
        angVel = diffAngles(targetAng, theta);
        amendDirection();
        if (linVel < distanceTol) {
            globalPath.erase(globalPath.begin());
            linVel = 0;
            angVel = diffAngles(goalAng,theta);
        }
        stringstream s;
        s << "Angles " << targetAng <<" "<< theta << " " << angVel;
        ROS_INFO("%s/n", s.str().c_str());
    } else if (dist > distanceTol) {
        linVel = distance(goal, loc);
        angVel = getAngle(goal, loc);
    } else if ( fabs(diffAngles(goalAng, theta)) > angleTol) {
        linVel = 0;
        angVel = diffAngles(goalAng, theta);
    } else {
        string msg = "Goal is reached!";
        ROS_INFO("%s/n", msg.c_str());
        std_msgs::Bool status_msg;
        status_msg.data = 1;
        statusPub.publish(status_msg);
        move = false;
        linVel = 0;
        angVel = 0;
    }
    // avoid turns with big radius, turn first, then move
    if (fabs(angVel) > M_PI/2.0 && linVel > 0) {
        linVel = 0;
    }
}

void Path::obstaclesCallback(const project_msgs::stop::ConstPtr& msg) {
    bool stop = msg->stop;
    if (stop) {
        move = false;
        string msg = "STOP!";
        ROS_INFO("%s/n", msg.c_str());
    }
}

void Path::amendDirection() {
    project_msgs::direction srv;
    srv.request.linVel = linVel;
    srv.request.angVel = angVel;
    if (lppService.call(srv)) {
        cout << "Direction changed from " << angVel;
        angVel = srv.response.angVel;
        cout << "  to " << angVel << endl;
    }
}


class GoalPosition {
  public:
    double x;
    double y;
    double theta;
    bool changedPosition;

    GoalPosition(GlobalPathPlanner& _gpp, Location& _loc, Path& _path):
                 x(0), y(0), theta(0),
                 changedPosition(false),
                 gpp(_gpp),
                 loc(_loc),
                 path(_path)
        {};
    void callback(const geometry_msgs::Twist::ConstPtr& msg);
    bool serviceCallback(project_msgs::global_path::Request &request,
                         project_msgs::global_path::Response &response);
  private:
    GlobalPathPlanner gpp;
    Location loc;
    Path path;
};

void GoalPosition::callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double x_new = msg->linear.x;
  double y_new = msg->linear.y;
  double theta_new = msg->angular.x;

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

}

bool GoalPosition::serviceCallback(project_msgs::global_path::Request &request,
                                   project_msgs::global_path::Response &response)
{
  double x_new = request.pose.linear.x;
  double y_new = request.pose.linear.y;
  double theta_new = request.pose.angular.x;

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
      pair<double, double> startCoord(loc.x,loc.y);
      pair<double, double> goalCoord(x,y);
      vector<pair<double,double> >  globalPath = gpp.getPath(startCoord, goalCoord);
      if (globalPath.size() == 0) {
          stringstream s;
          s << "Cant find a global path! Location " << loc.x <<" "<< loc.y;
          ROS_INFO("%s/n", s.str().c_str());
          response.path_found = false;
      } else {
          stringstream s;
          s << "Path is found, size" << globalPath.size();
          ROS_INFO("%s/n", s.str().c_str());
          path.setPath(x, y, theta, globalPath);
          changedPosition = false;
          response.path_found = true;
      }
  }
  return true;

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

  string mapFile = getHomeDir()+"/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
  GlobalPathPlanner gpp(mapFile, 0.01, 0.13);
  Location loc(0.215,0.224, M_PI/2.0);
  ros::Subscriber locationSub = n.subscribe("/odom", 1000, &Location::callback, &loc);
  Path path;
  path.lppService = n.serviceClient<project_msgs::direction>("local_path");
  path.statusPub = n.advertise<std_msgs::Bool>("navigation/status", 1);
  GoalPosition goal = GoalPosition(gpp, loc, path);
  //ros::Subscriber goalSub = n.subscribe("navigation/set_the_goal", 1000, &GoalPosition::callback, &goal);
  ros::ServiceServer service = n.advertiseService("navigation/set_the_goal", &GoalPosition::serviceCallback, &goal);

  ros::Subscriber subObstacles = n.subscribe("navigation/obstacles", 1000, &Path::obstaclesCallback, &path);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
  ros::Rate loop_rate(10);


  MapVisualization mapViz(gpp);
  stringstream s;
  s << "Grid Size " << gpp.gridSize.first <<" "<< gpp.gridSize.second << " Scale "<< gpp.mapScale.first << " cell "<< gpp.cellSize;
  ROS_INFO("%s/n", s.str().c_str());


  int count = 0;
  while (ros::ok())
  {

    if (path.move) {
        path.followPath(loc.x,loc.y,loc.theta);
        stringstream s;
        s << "Follow path " << path.linVel << " " << path.angVel;
        ROS_INFO("%s/n", s.str().c_str());
    } else {
      path.linVel = 0;
      path.angVel = 0;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = path.linVel;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = path.angVel;

    //ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);

    mapViz.publishMap();
    mapViz.publishPath(path.globalPath);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}


