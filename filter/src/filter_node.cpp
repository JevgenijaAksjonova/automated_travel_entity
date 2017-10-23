#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <random>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class FilterPublisher{
public:
    ros::NodeHandle n;
    ros::Publisher filter_publisher;
    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    double xpos;
    double ypos;
    double theta;
    double pi;
    std::vector<double> dphi_dt;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber lidar_subscriber;
    std::vector<float> ranges;
    float angle_increment;
    float range_min;
    float range_max;

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    struct Particle{
        double xPos;
        double yPos;
        double thetaPos;
        double weight;
    };

    Particle::Particle(double x, double y, double theta): xPos(x), yPos(y), thetaPos(theta) {}




    FilterPublisher(int frequency){
        control_frequency = frequency;
        n = ros::NodeHandle("~");
        pi = 3.1416;
        xpos = 0;
        ypos = 0;
        theta = 0;


        encoding_abs_prev = std::vector<int>(2,0);
        encoding_abs_new = std::vector<int>(2,0);
        encoding_delta = std::vector<double>(2,0);
        dphi_dt = std::vector<double>(2, 0.0);

        //set up random
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);

        first_loop = true;

        filter_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
        encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &OdometryPublisher::encoderCallbackLeft, this);
        encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &OdometryPublisher::encoderCallbackRight, this);
        lidar_subscriber = nh.subscribe("/scan", 1, &lidarCallback, this);


        wheel_r = 0.04;
        base_d = 0.25;
        tick_per_rotation = 900;
        control_frequenzy = 10; //10 hz
        dt = 1/control_frequenzy;

        k_D=1;
        k_V=1;
        k_W=1;


    }


    void encoderCallbackLeft(const phidgets::motor_encoder::ConstPtr& msg){
        encoding_abs_new[0] = msg->count;


    }

    void encoderCallbackRight(const phidgets::motor_encoder::ConstPtr& msg){
        encoding_abs_new[1] = -(msg->count);

    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        ranges = msg->ranges;
        angle_increment = msg->angle_increment;

        range_min = msg->range_min;
        range_max = msg->range_max;
    }


    void calculateVelocityAndNoise(){


        ros::Time current_time = ros::Time::now();


        if(first_loop){
            encoding_abs_prev[0] = encoding_abs_new[0];
            encoding_abs_prev[1] = encoding_abs_new[1];
        }
        first_loop =false;

        encoding_delta[0] = encoding_abs_new[0] - encoding_abs_prev[0];
        encoding_delta[1] = encoding_abs_new[1] - encoding_abs_prev[1];

        encoding_abs_prev[0] = encoding_abs_new[0];
        encoding_abs_prev[1] = encoding_abs_new[1];

        dphi_dt[0] = ((encoding_delta[0])/(tick_per_rotation)*2*pi)/dt;
        dphi_dt[1] = ((encoding_delta[1])/(tick_per_rotation)*2*pi)/dt;



        linear_v = (wheel_r/2)*(dphi_dt[1] + dphi_dt[0]);
        angular_w = (wheel_r/base_d)*(dphi_dt[1] - dphi_dt[0]);

        dist_D = std::normal_distribution<double>(0.0 (linear_v*dt*k_D)**2);
        dist_V = std::normal_distribution<double>(0.0 (linear_v*dt*k_V)**2);
        dist_W = std::normal_distribution<double>(0.0 (angular_w*dt*k_W)**2);


    }
    void updateParticlePosition(Particle p){

        double noise_D = dist_D(generator);
        double noise_V = dist_V(generator);
        double noise_W = dist_W(generator);

        p.xPos += (linear_v*dt + noise_D)*cos(theta);
        p.yPos += (linear_v*dt + noise_D)*sin(theta);
        p.thetaPos += (angular_w*dt + noise_W) + noise_V;

        if(p.thetaPos > pi){
            p.thetaPos = p.thetaPos-2*pi;
        }
        if(p.thetaPos<-pi){
            p.thetaPos = p.thetaPos+2*pi;
        }
    }

private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<double> encoding_delta;
    std::default_random_engine generator;
    std::normal_distribution dist_D;
    std::normal_distribution dist_V;
    std::normal_distribution dist_W;

    double wheel_r = 0.04;
    double base_d = 0.25;
    int tick_per_rotation = 900;
    double control_frequenzy = 10; //10 hz
    double dt = 1/control_frequenzy;
    int control_frequency;
    bool first_loop;
    double linear_v;
    double angular_w;

    double k_D;
    double k_V;
    double k_W;


};


int main(int argc, char **argv)
{

    double frequency = 10;
    ros::init(argc, argv, "filter_publisher");

    FilterPublisher filter(frequency);

    ROS_INFO("Spin!");

    ros::Rate loop_rate(frequency);

    int count = 0;
    while (odom.n.ok()){

        filter.calculateNewPosition();
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
