#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <memory>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <global_path_planner.h>
#include <map_visualization.h>

using namespace std;

MapVisualization::MapVisualization(shared_ptr<GlobalPathPlanner> _gpp):
    gpp(_gpp)
{

    n = ros::NodeHandle("~");
    grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/visualize_grid", 1);
    path_pub = n.advertise<nav_msgs::Path>("/visualize_path", 1);
    direction_pub = n.advertise<visualization_msgs::Marker>("/visualize_direction", 1);
    nodes_pub = n.advertise<visualization_msgs::MarkerArray>("/visualize_nodes_to_visit", 1);
    loadMap();

}

void MapVisualization::loadMap() {

    grid.header.frame_id = "/world_map";
    grid.header.stamp = ros::Time::now();

    grid.info.map_load_time = ros::Time::now();
    grid.info.resolution = gpp->cellSize;
    grid.info.width = gpp->gridSize.first;
    grid.info.height = gpp->gridSize.second;
    grid.info.origin.position.x = gpp->mapOffset.first;
    grid.info.origin.position.y = gpp->mapOffset.second;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    int id = 0;
    // visualize walls
    //int8 data[250*250];
    grid.data.clear();
    size_t nx = gpp->gridSize.first;
    size_t ny = gpp->gridSize.second;
    for (size_t j = 0; j < ny; j++) {
        for (size_t i = 0; i < nx; i++) {


            if (gpp->map[i][j] == 0) {
                grid.data.push_back(0);
            } else {
                grid.data.push_back(255);
            }
        }
    }

}



void MapVisualization::publishMap(int count) {

    loadMap();
    grid_pub.publish(grid);
}

void MapVisualization::publishPath(vector<pair<double, double> >& globalPath) {

    nav_msgs::Path path;
    path.header.frame_id = "/world_map";
    path.header.stamp = ros::Time::now();
    for (int i = 0; i < globalPath.size(); i++) {
        geometry_msgs::PoseStamped pose;
        //pair<int, int> cell = gpp->getCell(globalPath[i].first, globalPath[i].second);
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

void MapVisualization::publishDirection(double linVel, double angVel) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.lifetime = ros::Duration(0.1);
    marker.ns = "direction";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, angVel);
    marker.pose.orientation.x = q[0];
    marker.pose.orientation.y = q[1];
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];
    marker.scale.x = linVel;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    direction_pub.publish(marker);
}

void MapVisualization::publishNodes() {

    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < gpp->nodes.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/world_map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.ns = "nodesToVisit";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = gpp->mapOffset.first + (gpp->nodes[i].x+0.5)*gpp->cellSize;
        marker.pose.position.y = gpp->mapOffset.second + (gpp->nodes[i].y+0.5)*gpp->cellSize;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        markers.markers.push_back(marker);
    }
    nodes_pub.publish(markers);
}
