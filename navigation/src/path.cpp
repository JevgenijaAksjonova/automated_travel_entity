
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <memory>

#include <path.h>
#include <project_msgs/stop.h>
#include "project_msgs/direction.h"

using namespace std;

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

double Path::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2*M_PI ;
    }
    while (angle <= - M_PI) {
        angle += 2*M_PI;
    }
    return angle;
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
    double angle = atan2(y,x);
    return normalizeAngle(angle);
}


double Path::diffAngles(double a, double b) {
    double diff = a-b;
    return normalizeAngle(diff);
}

void Path::followPath(double x, double y, double theta) {
    pair<double, double> loc(x,y);
    pair<double,double> goal(goalX,goalY);
    double dist = distance(goal,loc);
    if (globalPath.size() > 0 ) {
        if (distance(globalPath[0],loc) > 1.1*pathRad) {
            move = false;
            string msg = "STOP! DEVIATION FROM THE PATH!";
            ROS_INFO("%s/n", msg.c_str());
        }
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
    if (fabs(angVel) > M_PI/4.0 && linVel > 0) {
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

