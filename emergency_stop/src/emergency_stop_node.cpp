#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"
#include "visualization_msgs/Marker.h"

class LidarListener
{
public:
    std::vector<float> ranges;
    float angle_increment;

    float range_min;
    float range_max;

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void LidarListener::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
    angle_increment = msg->angle_increment;

    range_min = msg->range_min;
    range_max = msg->range_max;
}


bool violation(float range, float angle) {

    float x = range * cos(angle);
    float y = range * sin(angle);

    float radius_x = 0.27;
    float radius_y = 0.17;

    float offset_x = 0.1;
    float offset_y = 0;

    float location = pow((x - offset_x), 2)/pow(radius_x, 2) +
                    pow((y - offset_y), 2)/pow(radius_y, 2);

    if(location <= 1) {
        ROS_INFO("Violation point [x, y, angle]: [%f, %f, %f]", x, y, angle);
        return true;
    }

    return false;
}


int number_violations(std::vector<float> ranges, float angle_increment) {
    int nr_violations = 0;
    float current_angle = M_PI;

    for(int i = 0; i < ranges.size(); i++) {
        if(!isinf(ranges[i])) {
            if(violation(ranges[i], current_angle)) {
                nr_violations++;
            }
        }
        current_angle -= angle_increment;
    }

    ROS_INFO("Number of violations: %d", nr_violations);

    return nr_violations;
}

bool danger(int threshold, std::vector<float> ranges, float angle_increment) {

    int violations = number_violations(ranges, angle_increment);

    if(violations > threshold) {
        return true;
    }
    return false;
}

void showRestrictedArea(ros::Publisher vis_pub) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time();
    marker.ns = "restricted_area";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.27*2;
    marker.scale.y = 0.17*2;
    marker.scale.z = 0.1;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    vis_pub.publish(marker);
}


int main(int argc, char **argv) {

    int violation_limit = 10;

    ros::init(argc, argv, "emergency_stop_node");

    ros::NodeHandle nh;

    ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1000);

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("restriction_marker", 0 );


    LidarListener lidar_listen;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1000, &LidarListener::callback, &lidar_listen);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        bool stop = false;

        if (danger(violation_limit, lidar_listen.ranges, lidar_listen.angle_increment)) {
            stop = true;
        }



        showRestrictedArea(vis_pub);

        std_msgs::Bool msg;

        msg.data = stop;

        stop_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
