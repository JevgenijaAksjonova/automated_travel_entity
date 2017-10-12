#include "ros/ros.h"
//#include <project_msgs/stop.h>
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "math.h"

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

    float radius_x = 0.2;
    float radius_y = 0.3;

    float offset_x = 0.1;
    float offset_y = 0;


    float location = pow((x - offset_x), 2)/pow(radius_x, 2) +
                    pow((y - offset_y), 2)/pow(radius_y, 2);

    if(location <= 1) {
        return true;
    }

    return false;
}


int number_violations(std::vector<float> ranges, float angle_increment) {
    int nr_violations = 0;
    int current_angle = 0;

    for(int i = 0; i < ranges.size(); i++) {
        if(!isinf(ranges[i])) {
            if(violation(ranges[i], current_angle)) {
                nr_violations++;
            }
        }
        current_angle += angle_increment;
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


int main(int argc, char **argv) {

    int violation_limit = 10;


    ros::init(argc, argv, "emergency_stop_node");

    ros::NodeHandle nh;

    ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("/emergency_stop", 1000);


    LidarListener lidar_listen;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1000, &LidarListener::callback, &lidar_listen);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        bool stop = false;


        if (danger(violation_limit, lidar_listen.ranges, lidar_listen.angle_increment)) {
            stop = true;
        }

        std_msgs::Bool msg;

        msg.data = stop;

        stop_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
