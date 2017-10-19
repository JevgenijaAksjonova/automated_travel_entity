#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <random>

#include <localization_global_map.h>


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


    while(ros::ok()) {
        localization_grid_pub.publish(map.visualGrid);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
