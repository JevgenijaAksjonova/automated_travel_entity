#include <ros/ros.h>
#include <project_msgs/stop.h>

class LidarListener
{
public:
    float32 ranges[];
    float32 angle_increment;

    float32 range_min;
    float32 range_max;

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void LidarListener::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
    angle_increment = msg->angle_increment;

    range_min = msg->range_min;
    range_max = msg->range_max;
}


bool inside_restriction(float32 range) {
    return false;
}


int number_violations() {
    int nr_violations = 0;

    for(int i = 0; i < sizeof(ranges); i++) {

        if(inside_restriction(ranges[i])) {
            nr_violations++;
        }
    }

    return nr_violations;
}

bool danger(int threshold) {

    int violations = number_violations();

    if(violations > threshold) {
        return true;
    }
    return false;
}


int main() {

    int violation_limit = 10;

    ros::init(argc, argv, "emergency_stop_node");

    ros::NodeHandle nh;

    //ros::Publisher stop_pub = nh.advertise<geometry_msgs::Twist>("/motor_controller/Twist", 1000);

    LidarListener lidar_listen;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1000, &LidarListener::callback, &lidar_listen);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        bool stop = false;


        if (danger(violation_limit)) {
            stop = true;
        }



        stop_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}
