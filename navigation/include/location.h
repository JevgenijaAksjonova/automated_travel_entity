#ifndef LOCATION_H
#define LOCATION_H 1

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class Location {
  public:
    double x;
    double y;
    double theta;

    Location() {};
    Location(double _xStart, double _yStart, double _thetaStart);
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
  private:
    double xStart;
    double yStart;
    double thetaStart;
};

#endif // LOCATION_H
