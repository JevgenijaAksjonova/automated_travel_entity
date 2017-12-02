#ifndef MAP_VISUALIZATION_H
#define MAP_VISUALIZATION_H 1

#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <memory>

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
    ros::Publisher direction_pub;
    ros::Publisher nodes_pub;
    std::shared_ptr<GlobalPathPlanner> gpp;
    nav_msgs::OccupancyGrid grid;
  public:
    MapVisualization(shared_ptr<GlobalPathPlanner> _gpp);
    void loadMap();
    void publishMap(int count);
    void publishPath(vector<pair<double, double> >& globalPath);
    void publishDirection(double linVel, double angVel);
    void publishNodes();

};

#endif // MAP_VISUALIZATION_H
