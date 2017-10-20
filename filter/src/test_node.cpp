#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <random>

#include <localization_global_map.h>
#include <measurements.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "world_map_node");

    std::string _filename_map = "/home/ras/catkin_ws/src/automated_travel_entity/world_map/maps/test.txt";
    float cellSize = 0.01;
    //std::string _filename_map;

    ros::NodeHandle n("~");
    ros::Rate r(10);

    LocalizationGlobalMap map(_filename_map, cellSize);

    ros::Publisher localization_grid_pub;

    localization_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/localization_grid", 1);

    pair<int, int> coord = getClosestWallCoordinates(map.global_map, map.cellSize, (M_PI/2), 20, 40);

    ROS_INFO("Closest wall [x, y]: [%d, %d]", coord.first, coord.second);

    pair<float, float> real_coord = map.getDistance(coord.first, coord.second);

    ROS_INFO("Distances [x, y]: [%f, %f]", real_coord.first, real_coord.second);

    while(ros::ok()) {
        localization_grid_pub.publish(map.visualGrid);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
