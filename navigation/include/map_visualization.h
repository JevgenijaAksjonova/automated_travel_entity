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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <global_path_planner.h>

using namespace std;

class MapVisualization {
  private:
    ros::NodeHandle n;
    ros::Publisher map_pub;
    GlobalPathPlanner gpp;
    visualization_msgs::MarkerArray global_walls;
    visualization_msgs::Marker wall;
    visualization_msgs::Marker path;
  public:
    MapVisualization(GlobalPathPlanner& _gpp);
    void loadMap();
    void publish();

};

#endif // MAP_VISUALIZATION_H
