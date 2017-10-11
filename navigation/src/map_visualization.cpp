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
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <global_path_planner.h>
#include <map_visualization.h>

using namespace std;

MapVisualization::MapVisualization(GlobalPathPlanner& _gpp){

    gpp = _gpp;
    n = ros::NodeHandle("~");
    grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/visualize_grid", 1);
    path_pub = n.advertise<nav_msgs::Path>("/visualize_path", 1);

/*    map_pub = n.advertise<visualization_msgs::MarkerArray>("visualize", 1);

    wall.header.frame_id = "/world_map";
    wall.header.stamp = ros::Time::now();

    wall.ns = "gpp_wall";
    wall.type = visualization_msgs::Marker::CUBE;
    wall.action = visualization_msgs::Marker::ADD;

    wall.pose.position.z = 0.1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    wall.scale.x = gpp.cellSize;
    wall.scale.y = gpp.cellSize;
    wall.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    wall.color.r = 1.0f;
    wall.color.g = 1.0f;
    wall.color.b = 0.0f;
    wall.color.a = 0.1;

    path.header.frame_id = "/world_map";
    path.header.stamp = ros::Time::now();

    path.ns = "gpp_wall";
    path.type = visualization_msgs::Marker::CUBE;
    path.action = visualization_msgs::Marker::ADD;

    path.pose.position.z = 0.1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    path.scale.x = gpp.cellSize;
    path.scale.y = gpp.cellSize;
    path.scale.z = 0.05;

    path.color.r = 0.0f;
    path.color.g = 0.0f;
    path.color.b = 1.0f;
    path.color.a = 0.5;
*/
    loadMap();

}

void MapVisualization::loadMap() {

    grid.header.frame_id = "/world_map";
    grid.header.stamp = ros::Time::now();

    grid.info.map_load_time = ros::Time::now();
    grid.info.resolution = gpp.cellSize;
    grid.info.width = gpp.gridSize.first;
    grid.info.height = gpp.gridSize.second;
    grid.info.origin.position.x = gpp.mapOffset.first;
    grid.info.origin.position.y = gpp.mapOffset.second;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    int id = 0;
    // visualize walls
    //int8 data[250*250];
    grid.data.clear();
    size_t nx = gpp.gridSize.first;
    size_t ny = gpp.gridSize.second;
    for (size_t j = 0; j < ny; j++) {
        for (size_t i = 0; i < nx; i++) {


            if (gpp.map[i][j] == 0) {
                grid.data.push_back(0);
            } else {
                grid.data.push_back(255);
            }
        }
    }

}

void MapVisualization::publishMap() {

    grid_pub.publish(grid);
}

void MapVisualization::publishPath(vector<pair<double, double> >& globalPath) {

    nav_msgs::Path path;
    path.header.frame_id = "/world_map";
    path.header.stamp = ros::Time::now();
    for (int i = 0; i < globalPath.size(); i++) {
        geometry_msgs::PoseStamped pose;
        //pair<int, int> cell = gpp.getCell(globalPath[i].first, globalPath[i].second);
        pose.pose.position.x = globalPath[i].first;
        pose.pose.position.y = globalPath[i].second;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        path.poses.push_back(pose);
    }
    path_pub.publish(path);
}

