#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <functional>
#include <random>
#include <chrono>
#include <ctime>
#include <stdlib.h>
#include <pwd.h>
#include <sstream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <localization_global_map.h>
#include <measurements.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;

class WallFinder
{
  public:
    ros::NodeHandle n;

    ros::Subscriber lidar_subscriber;
    ros::Subscriber position_subscriber;
    std::vector<float> ranges;
    float angle_increment;
    float range_min;
    float range_max;
    int _nr_measurements;
    float _xPos;
    float _yPos;
    float _thetaPos;

    std::vector<pair<float, float>> outliers;

    //LocalizationGlobalMap map;

    WallFinder()
    {
        n = ros::NodeHandle("~");
        int nr_measurements = 8;
        _xPos = 0;
        _yPos = 0;
        _thetaPos = 0;


        if(!n.getParam("/wall_finder/wall_finder_params/nr_measurements",nr_measurements)){
            ROS_ERROR("failed to detect parameter");
            exit(EXIT_FAILURE);
        }


        ROS_INFO("Running filter with parameters:");
        ROS_INFO("Number of measurements: [%d]", nr_measurements);

        lidar_subscriber = n.subscribe("/scan", 1, &WallFinder::lidarCallback, this);
        position_subscriber = n.subscribe("/odom", 1, &WallFinder::positionCallback, this );

        _nr_measurements = nr_measurements;
    }


    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        ranges = msg->ranges;
        angle_increment = msg->angle_increment;

        range_min = msg->range_min;
        range_max = msg->range_max;
    }

    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {

        _xPos = msg->pose.pose.position.x;//xStart - msg->pose.pose.position.y;
        _yPos = msg->pose.pose.position.y;//yStart + msg->pose.pose.position.x;

        geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
        _thetaPos = tf::getYaw(odom_quat);

    }
    
    void lookForWalls(LocalizationGlobalMap map){
        if(_xPos == 0 || _yPos == 0 || _thetaPos == 0){
            return;
        }


    }

  private:

    std::vector<float> prob_meas;
    float OUTLIER_THRESHOLD = 0.2;

};

int main(int argc, char **argv)
{

    ROS_INFO("Spin!");

    float frequency = 10;
    struct passwd *pw = getpwuid(getuid());
    std::string homePath(pw->pw_dir);

    std::string _filename_map = homePath+"/catkin_ws/src/automated_travel_entity/filter/maps/lab_maze_2017.txt";
    float cellSize = 0.01;

    ros::init(argc, argv, "wall_finder_publisher");


    WallFinder wf;

    LocalizationGlobalMap map(_filename_map, cellSize);

    ros::Rate loop_rate(frequency);

    Particle most_likely_position;
    Particle most_likely_position_prev;
    most_likely_position_prev.xPos = 0.0;
    most_likely_position_prev.yPos = 0.0;
    most_likely_position_prev.thetaPos = 0.0;
    std::vector<std::pair<float, float>> sampled_measurements;
    bool ready_to_run = false;
    ROS_INFO("Filter waiting for initial position");




    int count = 0;
    while (wf.n.ok())
    {
        wf.lookForWalls(map);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
