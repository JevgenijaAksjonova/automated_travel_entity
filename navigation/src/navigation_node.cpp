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
#include <project_msgs/stop.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <math.h>
#include <iostream>

#include <global_path_planner.h>
#include <map_visualization.h>

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
  ROS_INFO("%s/n", s.str().c_str());
}


class GoalPosition {
  public:
    double x;
    double y;
    double theta;
    bool changedPosition;

    GoalPosition(): x(0), y(0), theta(0), changedPosition(false) {};
    void callback(const geometry_msgs::Twist::ConstPtr& msg);
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

class Path {
  public:
    double linVel;
    double angVel;
    bool move;

    vector<pair<double,double> > globalPath;

    Path(): linVel(0), angVel(0), pathRad(0.10), distanceTol(0.05), angleTol(2*M_PI/45.0), move(false) {};
    void setGoal(double x, double y, double theta);
    void followPath(double x, double y, double theta);
    void obstaclesCallback(const project_msgs::stop::ConstPtr& msg);
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
};

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
    if (diff > M_PI) {
        return diff - 2*M_PI ;
    } else if (diff <= - M_PI) {
        return diff + 2*M_PI;
    }
    return diff;
}

void Path::followPath(double x, double y, double theta) {
    pair<double, double> loc(x,y);
    pair<double,double> goal(goalX,goalY);
    double dist = distance(goal,loc);
    if (globalPath.size() > 0 ) {
        while (globalPath.size() > 1 && distance(globalPath[0],loc) < pathRad) {
            globalPath.erase(globalPath.begin());
        }
        linVel = distance(globalPath[0], loc);
        double targetAng = getAngle(globalPath[0],loc);
        angVel = diffAngles(targetAng, theta);
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
        move = false;
        linVel = 0;
        angVel = 0;
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle n;

  string mapFile = "/home/ras13/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
  GlobalPathPlanner gpp(mapFile, 0.02, 0.15);
  Location loc(0.215,0.224, M_PI/2.0);
  ros::Subscriber locationSub = n.subscribe("/odom", 1000, &Location::callback, &loc);
  GoalPosition goal = GoalPosition();
  ros::Subscriber goalSub = n.subscribe("navigation/set_the_goal", 1000, &GoalPosition::callback, &goal);
  Path path;
  ros::Subscriber subObstacles = n.subscribe("navigation/obstacles", 1000, &Path::obstaclesCallback, &path);
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist", 1000);
  ros::Rate loop_rate(10);

  //MapVisualization mapViz(gpp);

  int count = 0;
  while (ros::ok())
  {

    if (goal.changedPosition) {
        string msg = "Recalculate path";
        ROS_INFO("%s/n", msg.c_str());
        pair<double, double> startCoord(loc.x,loc.y);
        pair<double, double> goalCoord(goal.x,goal.y);
        path.globalPath = gpp.getPath(startCoord, goalCoord);
        if (path.globalPath.size() == 0) {
            stringstream s;
            s << "Cant find a global path! Location " << loc.x <<" "<< loc.y;
            ROS_INFO("%s/n", s.str().c_str());
        } else {
            stringstream s;
            s << "Path is found, size" << path.globalPath.size();
            ROS_INFO("%s/n", s.str().c_str());
            path.setGoal(goal.x, goal.y, goal.theta);
            goal.changedPosition = false;
            path.move = true;
        }
    }

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

    //mapViz.publish();
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}


