#ifndef MAP_VISUALIZATION_H
#define MAP_VISUALIZATION_H 1

#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <global_path_planner.h>

using namespace std;

class MapVisualization {
  private:
    ros::NodeHandle n;
    ros::Publisher grid_pub;
    ros::Publisher path_pub;
    GlobalPathPlanner gpp;
    nav_msgs::OccupancyGrid grid;
  public:
    MapVisualization(GlobalPathPlanner& _gpp);
    void loadMap();
    void publishMap();
    void publishPath(vector<pair<double, double> >& globalPath);

};

#endif // MAP_VISUALIZATION_H
