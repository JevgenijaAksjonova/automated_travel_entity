#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <project_msgs/stop.h>

class ADCListener
{
public:
    int dist_sensor1;
    int dist_sensor2;

    void callback(const ras_lab1_msgs::ADConverter::ConstPtr& msg);
};

void ADCListener::callback(const ras_lab1_msgs::ADConverter::ConstPtr& msg)
{
    dist_sensor1 = msg->ch1;
    dist_sensor2 = msg->ch2;
}


int main() {




    return 0;
}
