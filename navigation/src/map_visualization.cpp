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
#include <map_visualization.h>

using namespace std;

MapVisualization::MapVisualization(GlobalPathPlanner& _gpp){

    gpp = _gpp;
    n = ros::NodeHandle("~");
    map_pub = n.advertise<visualization_msgs::MarkerArray>("visualize", 1);

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

    loadMap();

}

void MapVisualization::loadMap() {

    int id = 0;
    // visualize walls
    size_t nx = gpp.gridSize.first;
    size_t ny = gpp.gridSize.second;
    for (size_t i = 0; i < nx; i++) {
        for (size_t j = 0; j < ny; j++) {

            if (gpp.map[i][j] == 0) {
                continue;
            }

            wall.pose.position.x = gpp.mapOffset.first + (i+0.5)*gpp.cellSize;
            wall.pose.position.y = gpp.mapOffset.second + (j+0.5)*gpp.cellSize;
            wall.id = id;
            id++;

            global_walls.markers.push_back(wall);
        }
    }
    //visualize path
/*    for (size_t k = 0; k < gpp.globalPath.size(); k++) {
        pair<int,int> point = gpp.globalPath[k];
        path.pose.position.x = gpp.mapOffset.first + (point.first+0.5)*gpp.cellSize;
        path.pose.position.y = gpp.mapOffset.secodn + (point.second+0.5)*gpp.cellSize;
        path.id = id;
        id++;

        global_walls.markers.push_back(path);
    }*/

}

void MapVisualization::publish() {
    map_pub.publish(global_walls);
}


