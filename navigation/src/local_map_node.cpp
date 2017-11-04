/*
 *  local_map_node.cpp
 *
 *
 *  Created on: Nov 1, 2017
 *  Authors:   Jevgenija Aksjonova
 *            jevaks <at> kth.se
 */

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "project_msgs/direction.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

class LocalPathPlanner {
  public:
    ros::Publisher lppViz;

    LocalPathPlanner(double p_robotRad, double p_mapRad):
                                    robotRad(p_robotRad),
                                    mapRad(p_mapRad),
                                    localMap(360,0) {};

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    bool amendDirection(project_msgs::direction::Request  &req,
                        project_msgs::direction::Response &res);
    void showLocalMap();
  private:
    double mapRad;
    double robotRad;

    // lidar data
    vector<float> ranges;
    float angleIncrement;
    float range_min;
    float range_max;

    vector<double> localMap;
    void updateLocalMapLidar();
    void addRobotRadius();
};

void LocalPathPlanner::addRobotRadius(){

    int angAdd = round(asin(robotRad/mapRad));
    for (int i = 0; i < localMap.size(); i++) {
        for (int j = i-angAdd; j < i+angAdd; j++) {
            if (localMap[j % 360] > 0) {
               localMap[i] = 1.0;
            }
        }
    }
}

void LocalPathPlanner::updateLocalMapLidar() {

    localMap = vector<double>(360,0);
    double angleLid = M_PI;
    double xOffset = 0.10;
    for (int i=0; i < ranges.size(); i++) {
        double x = ranges[i]*cos(angleLid) + xOffset;
        double y = ranges[i]*sin(angleLid);
        double r = pow(pow(x,2)+pow(y,2),0.5);
        double angle = atan2(y,x) + M_PI;
        int angleInd = round(angle/2.0/M_PI *360);
        if (r <= mapRad) {
            localMap[angleInd] = 1.0;
        }
        angleLid -= angleIncrement;
    }

}

void LocalPathPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
    angleIncrement = msg->angle_increment;

    range_min = msg->range_min;
    range_max = msg->range_max;

    updateLocalMapLidar();
}

bool LocalPathPlanner::amendDirection(project_msgs::direction::Request  &req,
                                      project_msgs::direction::Response &res) {

    addRobotRadius();

    int angleInd = round(req.angVel/2.0/M_PI*360);
    int angleIndLeft = angleInd;
    int angleIndRight = angleInd;
    while (localMap[angleIndLeft % 360] > 0 && abs(angleIndLeft - angleInd) <= 180) {
        angleIndLeft--;
    }
    while (localMap[angleIndRight % 360] > 0 && abs(angleIndRight - angleInd) <= 180) {
        angleIndRight++;
    }
    if (abs(angleIndLeft - angleInd) >= abs(angleIndRight + angleInd)) {
        res.angVel = angleIndRight/360.0*2*M_PI;
    } else {
        res.angVel = angleIndLeft/360.0*2*M_PI;
    }
    return true;

}

void LocalPathPlanner::showLocalMap() {

    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < localMap.size(); i++) {
        if (localMap[i] > 0) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/base_link";
            marker.header.stamp = ros::Time::now();
            marker.ns = "local_map";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = localMap[i]*cos(2.0*M_PI*i/360);
            marker.pose.position.y = localMap[i]*sin(2.0*M_PI*i/360);
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            markers.markers.push_back(marker);
        }
    }
    lppViz.publish(markers);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;

    LocalPathPlanner lpp(0.13, 0.20);
    ros::ServiceServer service = nh.advertiseService("local_path", &LocalPathPlanner::amendDirection, &lpp);
    ros::Subscriber lidarSub = nh.subscribe("/scan", 1000, &LocalPathPlanner::lidarCallback, &lpp);

    lpp.lppViz = nh.advertise<visualization_msgs::MarkerArray>("navigation/visualize_lpp", 1);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
/*
        if (danger(violation_limit, lidar_listen.ranges, lidar_listen.angle_increment)) {
            stop = true;
        }


        showRestrictedArea(vis_pub);

        std_msgs::Bool msg;

        msg.data = stop;

        stop_pub.publish(msg);
*/
        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
