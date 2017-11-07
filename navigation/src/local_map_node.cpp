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

int mod(int a, int b) {
    while (a < 0) a +=b;
    return a % b;
}

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
    void addRobotRadius(vector<double>& localMap);
    void filterNoise(vector<double>& localMap);
};

void LocalPathPlanner::addRobotRadius(vector<double>& localMap){

    vector<double> localMapNew = localMap;
    int angAdd = round(asin(robotRad/mapRad)/2.0/M_PI*360);
    for (int i = 0; i < localMap.size(); i++) {
        for (int j = i-angAdd; j < i+angAdd; j++) {
            if (localMap[mod(j,360)] > 0) {
               localMapNew[i] = 1.0;
            }
        }
    }
    localMap = localMapNew;
}

void LocalPathPlanner::filterNoise(vector<double>& localMap){

    vector<double> localMapNew = localMap;
    int w = 5; // window width = 2*w +1
    for (int i = 0; i < localMap.size(); i++) {
        if (localMap[i] > 0) {
            int count = 0;
            for (int j = i-w; j <= i+w; j++) {
                if (localMap[mod(j,360)] > 0) {
                   count += 1;
                }
            }
            if (count < w+1) {
                localMapNew[i] = 0.0;
            }
        }
    }
    localMap = localMapNew;
}


void LocalPathPlanner::updateLocalMapLidar() {

    //vector<double> localMapNew(360,0);
    vector<double> localMapNew = localMap;
    double angleLid = 0.0;
    double xOffset = 0.05;
    for (int i=0; i < ranges.size(); i++) {
        if (!isinf(ranges[i]) ) {
            double x = ranges[i]*cos(angleLid) + xOffset;
            double y = ranges[i]*sin(angleLid);
            double r = pow(pow(x,2)+pow(y,2),0.5);
            double angle = atan2(y,x);
            int angleInd = round(angle/2.0/M_PI *360);
            angleInd = mod(angleInd,360);
            if (r <= mapRad) {
                localMapNew[angleInd] = 1.0;
            } else {
                localMapNew[angleInd] = 0.0;
            }
        }
        angleLid += angleIncrement;
    }
    filterNoise(localMapNew);
    addRobotRadius(localMapNew);
    localMap = localMapNew;
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


    for (int i = 0; i < localMap.size(); i++) {
        cout << localMap[i] << " ";
    }
    cout << endl;

    int angleInd = round(req.angVel/2.0/M_PI*360);
    int angleIndLeft = angleInd;
    int angleIndRight = angleInd;
    while (localMap[mod(angleIndLeft,360)] > 0 && abs(angleIndLeft - angleInd) <= 180) {
        angleIndLeft--;
    }
    while (localMap[mod(angleIndRight,360)] > 0 && abs(angleIndRight - angleInd) <= 180) {
        angleIndRight++;
    }
    if (abs(angleIndLeft - angleInd) >= abs(angleIndRight + angleInd)) {
        res.angVel = angleIndRight/360.0*2*M_PI;
    } else {
        res.angVel = angleIndLeft/360.0*2*M_PI;
    }
    cout << "angleInd " << angleInd << ", right " << angleIndRight << ", left " << angleIndLeft << endl;
    return true;

}

void LocalPathPlanner::showLocalMap() {

    //cout<< "Local Map: " ;
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < localMap.size(); i++) {
        //cout << localMap[i] ;
        if (localMap[i] > 0) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = i;
            marker.lifetime = ros::Duration(0.1);
            marker.ns = "local_map";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cos(i/360.0*2.0*M_PI);
            marker.pose.position.y = sin(i/360.0*2.0*M_PI);
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
    //cout << endl;
    lppViz.publish(markers);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;

    LocalPathPlanner lpp(0.13, 0.35);
    ros::ServiceServer service = nh.advertiseService("local_path", &LocalPathPlanner::amendDirection, &lpp);
    ros::Subscriber lidarSub = nh.subscribe("/scan", 1000, &LocalPathPlanner::lidarCallback, &lpp);

    lpp.lppViz = nh.advertise<visualization_msgs::MarkerArray>("navigation/visualize_lpp", 360);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        lpp.showLocalMap();
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
