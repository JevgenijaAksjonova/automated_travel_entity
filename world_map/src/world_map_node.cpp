
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "world_map_node");

    std::string _filename_map = "/home/oskar/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
    //std::string _filename_map;

    ros::NodeHandle n("~");
    ros::Rate r(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

    n.param<std::string>("filename_map", _filename_map);

    visualization_msgs::MarkerArray global_walls;
    visualization_msgs::Marker wall;

    wall.header.frame_id = "/odom";
    wall.header.stamp = ros::Time::now();

    wall.ns = "global_wall";
    wall.type = visualization_msgs::Marker::CUBE;
    wall.action = visualization_msgs::Marker::ADD;

    wall.pose.position.z = 0.1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    //marker.scale.x = 1.0;
    wall.scale.y = 0.01;
    wall.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    wall.color.r = 0.0f;
    wall.color.g = 1.0f;
    wall.color.b = 0.0f;
    wall.color.a = 1.0;

    std::ifstream file;
    file.open(_filename_map.c_str());


    if (file.is_open()){
        std::cout << "File is open." << std::endl;
        std::string line;

        int id = 0;
        while(std::getline(file, line)) {

            if (line[0] == '#' || line.empty()) {
                continue;
            }



            double x1, y1, x2, y2;

            std::istringstream ss(line);

            ss >> x1 >> y1 >> x2 >> y2;

            double centre_x = (x1 + x2) / 2;
            double centre_y = (y1 + y2) / 2;

            wall.pose.position.x = centre_x;
            wall.pose.position.y = centre_y;

            double rotation = atan2((y2 - y1), (x2 - x1));
            double length = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

            tf::Quaternion q;
            q.setRPY(0.0, 0.0, rotation);
            tf::quaternionTFToMsg(q, wall.pose.orientation);

            wall.scale.x = length;


            std::cout << "x1: " << x1 << std::endl;
            std::cout << "y1: " << y1 << std::endl;

            std::cout << "x2: " << x2 << std::endl;
            std::cout << "y2: " << y2 << std::endl << std::endl;

            wall.id = id;
            id++;

            global_walls.markers.push_back(wall);
        }

        file.close();
        std::cout << "File closed." << std::endl;


        while(ros::ok()) {
            marker_pub.publish(global_walls);
            ros::spinOnce();
            r.sleep();
        }
    }
    else std::cout << "Unable to open the file" << std::endl;



    return 0;
}
