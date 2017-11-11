#ifndef NAVIGATION_PATH_H
#define NAVIGATION_PATH_H 1

#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <memory>

#include <project_msgs/stop.h>
#include "project_msgs/direction.h"

using namespace std;

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

#endif // NAVIGATION_PATH_H
