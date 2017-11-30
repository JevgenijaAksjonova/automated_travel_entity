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
#include "project_msgs/stop.h"
#include "project_msgs/depth.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>

using namespace std;

int mod(int a, int b) {
    while (a < 0) a +=b;
    return a % b;
}

class LocalPathPlanner {
  public:
    ros::Publisher lppViz;
    ros::Publisher stopPub;

    LocalPathPlanner(double p_robotRad, double p_mapRad):
                                    robotRad(p_robotRad),
                                    mapRad(p_mapRad),
                                    localMap(360,0),
                                    distance(360,0),
                                    dConf(0.02),
                                    useDepth(true){};

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void depthCallback(const project_msgs::depth::ConstPtr& msg);
    void locationCallback(const nav_msgs::Odometry::ConstPtr& msg);
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

    // depth data
    bool useDepth;
    vector<float> rangesDepth;
    vector<float> anglesDepth;
    vector<float> confDepth;
    double dConf;

    //location
    double locX;
    double locY;
    double locTheta;

    vector<double> localMap;
    vector<double> localMapProcessed;
    vector<double> distance;
    void updateLocalMapLidar();
    void addRobotRadius(vector<double>& localMap);
    void filterNoise(vector<double>& localMap);
    void addDepth(vector<double>& localMap);

    void transform(float &r, float &a, float &dr);

    void stop(int reason);
    void emergencyStopLidar();
};

void LocalPathPlanner::stop(int reason) {
    project_msgs::stop msg;
    msg.stamp = ros::Time::now();
    msg.stop = true;
    msg.reason = reason;
    stopPub.publish(msg);
}

void LocalPathPlanner::addRobotRadius(vector<double>& localMap){

    vector<double> localMapNew(localMap);
    //cout<< "ang add = ";
    for (int i = 0; i < localMap.size(); i++) {
        if (localMap[i] > 0) {

            double d = distance[i];
            int angAddMax = (asin((robotRad)/max(d,robotRad))/2.0/M_PI*360);
            int angAddMin = (asin((robotRad-0.05)/max(d,robotRad-0.05))/2.0/M_PI*360);
            //cout << "(" << angAddMin << ":"<<angAddMax << ")" ;
            for (int di = -angAddMax; di < angAddMax+1; di++) {
                int j = i + di;
                double value;
                if (abs(di) <= angAddMin) {
                    value = 1;
                } else {
                    value = (angAddMax + 1 - abs(di))/(float)(angAddMax+1-angAddMin);
                }
                localMapNew[mod(j,360)] = max(localMapNew[mod(j,360)],value);
            }
        }
    }
    //cout << endl;
    localMap = localMapNew;
}

void LocalPathPlanner::filterNoise(vector<double>& localMap){

    vector<double> localMapNew(localMap);
    int w = 3; // window width = 2*w +1
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

void LocalPathPlanner::addDepth(vector<double>& localMap){

    vector<double> localMapNew(localMap);
    int l = anglesDepth.size();
    for (int i = 0; i < l; i++) {
        double r = rangesDepth[i];
        int ind = mod(round(anglesDepth[i]/2.0/M_PI*360),360);
        if (r <= mapRad) {
            localMapNew[ind] = max(localMapNew[ind], 1.0);
        }
        distance[ind] = min(distance[ind], r);
        //cout << ind << " " << r << " "<< distance[ind]<< endl;
    }
    localMap = localMapNew;
}


void LocalPathPlanner::updateLocalMapLidar() {

    //vector<double> localMapNew(360,0);
    vector<double> localMapNew = localMap;
    double angleLid = -M_PI/2.0;
    double xOffset = -0.03;
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
                //cout << angleInd << " " << r << endl;
            } else {
                localMapNew[angleInd] = 0.0;
            }
            distance[angleInd] = r;
        }
        angleLid += angleIncrement;
    }
    localMap = localMapNew;
    //cout << "LOCAL MAP Distance " << endl;
    //for (int i = 0; i < localMapNew.size(); i++) {
    //    cout << localMapNew[i];// << ":" <<distance[i] <<" ";
    //}
    //cout << endl;
    //cout << "LOCAL MAP NEW " << endl;
    //for (int i = 0; i < localMapNew.size(); i++) {
    //    cout << localMapNew[i] << " ";
    //}
    //cout << endl;
    filterNoise(localMapNew);
    //cout << "LOCAL MAP FILTERED" << endl;
    //for (int i = 0; i < localMapNew.size(); i++) {
    //    cout << localMapNew[i] << " ";
    //}
    //cout << endl;
    addDepth(localMapNew);
    //cout << "LOCAL MAP DEPTH" << endl;
    //for (int i = 0; i < localMapNew.size(); i++) {
    //    cout << localMapNew[i] ;//<<":" <<distance[i] <<" ";
    //}
    //cout << endl;
    addRobotRadius(localMapNew);
    //cout << "LOCAL MAP RADIUS" << endl;
    //for (int i = 0; i < localMapNew.size(); i++) {
    //    cout << localMapNew[i] << " ";
    //}
    //cout << endl;
    localMapProcessed = localMapNew;
    //cout << "LOCAL MAP" << endl ;
    //for (int i = 0; i < localMap.size(); i++) {
    //    cout << localMap[i] <<" ";
    //}
    //cout << endl;
}

void LocalPathPlanner::emergencyStopLidar() {
    //cout << "RANGE "<< endl;
    int count = 0;
    for (int i=60; i < 121; i++) {
        //cout << ranges[i] << " ";
        if (  ranges[i]< 0.215) {
            count++;
        }
    }
    //cout << endl;
    //cout << count << endl;
    if (count > 2) {
        stop(1);
        stringstream s;
        s << "EMERGENCY STOP, LIDAR! ";
        for (int i=60; i < 121; i++) {
            if (  ranges[i]< 0.215) {
                s << i <<" ";
            }
        }
        ROS_INFO("%s/n", s.str().c_str());
    }
}

void LocalPathPlanner::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
    angleIncrement = msg->angle_increment;

    range_min = msg->range_min;
    range_max = msg->range_max;

    emergencyStopLidar();
    updateLocalMapLidar();
}

void LocalPathPlanner::depthCallback(const project_msgs::depth::ConstPtr& msg) {
    //rangesDepth = msg->ranges;
    //anglesDepth = msg->angles;
    
    int i = 0;
    while(i < rangesDepth.size()) {
        confDepth[i] -= dConf;
        if (confDepth[i] <= 0) {
            rangesDepth.erase(rangesDepth.begin()+i);
            anglesDepth.erase(anglesDepth.begin()+i);
            confDepth.erase(confDepth.begin()+i);
        } else {
            i++;
        }
    }
    rangesDepth.insert(rangesDepth.end(), msg->ranges.begin(), msg->ranges.end());
    anglesDepth.insert(anglesDepth.end(), msg->angles.begin(), msg->angles.end());
    vector<float> newConf(msg->ranges.size(),1.0); 
    confDepth.insert(confDepth.end(), newConf.begin(), newConf.end());
    cout << "DATA FROM DEPTH " << cout;
    for(int i =0; i < rangesDepth.size(); i++) {
        cout << i<< ": "<< rangesDepth[i] << " " << anglesDepth[i] << " "<< confDepth[i]<< endl;
    }
}

void LocalPathPlanner::transform(float &r, float &a, float &dr) {
    // cosine law
    r = pow(pow(r,2) + pow(dr,2) - 2*dr*r*cos(a),0.5);
    // sine law
    a += asin(sin(a)*dr/r);
}

void LocalPathPlanner::locationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double locX_new = msg->pose.pose.position.x;//xStart - msg->pose.pose.position.y;
    double locY_new = msg->pose.pose.position.y;//yStart + msg->pose.pose.position.x;

    geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
    double locTheta_new = tf::getYaw(odom_quat);

    double dx = locX_new - locX;
    double dy = locY_new - locY;
    double dtheta = locTheta_new - locTheta;
    locX = locX_new;
    locY = locY_new;
    locTheta = locTheta_new;

    cout << "Location prev "<< locX << " " << locY << " "<< locTheta <<endl;
    cout << "Location New " << locX_new << " " << locY_new << " " << locTheta_new << endl;
    cout << "Deltas "<< dx << " " << dy << " " << dtheta << endl;

    // check if robot moved forward
    double diff = atan2(dy,dx) - locTheta_new;
    while (diff> M_PI) {
        diff -= 2*M_PI ;
    }
    while (diff <= - M_PI) {
        diff += 2*M_PI;
    }
    cout << "Diff = "<< diff << endl;
    if (fabs(diff) < M_PI/3.0) {
        float dr = pow(dx*dx+dy*dy,0.5);
        cout << "dr = " << dr << endl;
        for (int i = 0; i < rangesDepth.size(); i++) {
            anglesDepth[i] -= dtheta;
            transform(rangesDepth[i], anglesDepth[i], dr);
        }
    }
}

bool LocalPathPlanner::amendDirection(project_msgs::direction::Request  &req,
                                      project_msgs::direction::Response &res) {


    mapRad = req.linVel;
    //updateLocalMapLidar();

    for (int i = 0; i < localMapProcessed.size(); i++) {
        cout << localMapProcessed[i] << " ";
    }
    cout << endl;

    int angleInd = round(req.angVel/2.0/M_PI*360);
    int angleIndLeft = angleInd;
    int angleIndRight = angleInd;
    while (localMapProcessed[mod(angleIndLeft,360)] > 0.3 &&
           localMapProcessed[mod(angleIndLeft,360)] >= localMapProcessed[mod(angleIndLeft-1,360)] &&
           abs(angleIndLeft - angleInd) <= 180) {
        angleIndLeft--;
    }
    while (localMapProcessed[mod(angleIndRight,360)] > 0.3 &&
           localMapProcessed[mod(angleIndRight,360)] >= localMapProcessed[mod(angleIndRight+1,360)] &&
           abs(angleIndRight - angleInd) <= 180) {
        angleIndRight++;
    }
    if (abs(angleIndLeft - angleInd) > 180 && abs(angleIndRight - angleInd) > 180) {
        angleIndLeft = angleInd;
        angleIndRight = angleInd;
        while (localMapProcessed[mod(angleIndLeft,360)] > 0.9 && abs(angleIndLeft - angleInd) <= 180) {
            angleIndLeft--;
        }
        while (localMapProcessed[mod(angleIndRight,360)] > 0.9 && abs(angleIndRight - angleInd) <= 180) {
            angleIndRight++;
        }
        if (abs(angleIndLeft - angleInd) > 180 && abs(angleIndRight - angleInd) > 180) {
            res.angVel = 0;
            stop(3);
        }
    }
    if (localMapProcessed[mod(angleInd,360)] < 1.0) {
        if (localMapProcessed[mod(angleIndLeft,360)]< localMapProcessed[mod(angleIndRight,360)]) {
            res.angVel = angleIndLeft/360.0*2*M_PI;
        } else {
            res.angVel = angleIndRight/360.0*2*M_PI;
        }
    } else {
        if (abs(angleIndLeft - angleInd) >= abs(angleIndRight + angleInd)) {
            res.angVel = angleIndRight/360.0*2*M_PI;
        } else {
            res.angVel = angleIndLeft/360.0*2*M_PI;
        }
    }
    cout << "angleInd " << angleInd << ", right " << angleIndRight << ", left " << angleIndLeft << endl;
    return true;

}

void LocalPathPlanner::showLocalMap() {

    //cout<< "Local Map: " ;
    visualization_msgs::MarkerArray markers;
    for (int i = 0; i < localMapProcessed.size(); i++) {
        //cout << localMap[i] ;
        if (localMapProcessed[i] > 0) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/base_link";
            marker.header.stamp = ros::Time::now();
            marker.id = i;
            marker.lifetime = ros::Duration(0.1);
            marker.ns = "local_map";
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = mapRad*cos(i/360.0*2.0*M_PI);
            marker.pose.position.y = mapRad*sin(i/360.0*2.0*M_PI);
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
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

    LocalPathPlanner lpp(0.18, 0.25);
    ros::ServiceServer service = nh.advertiseService("local_path", &LocalPathPlanner::amendDirection, &lpp);
    ros::Subscriber lidarSub = nh.subscribe("/scan", 1, &LocalPathPlanner::lidarCallback, &lpp);
    ros::Subscriber depthSub = nh.subscribe("/depth", 1, &LocalPathPlanner::depthCallback, &lpp);

    // Visualize
    lpp.lppViz = nh.advertise<visualization_msgs::MarkerArray>("navigation/visualize_lpp", 360);

    // STOP!
    lpp.stopPub = nh.advertise<project_msgs::stop>("navigation/obstacles", 1);
    ros::Rate loop_rate(10);

    // Location
    ros::Subscriber locationSub = nh.subscribe("/odom", 1, &LocalPathPlanner::locationCallback, &lpp);

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
