#include "ros/ros.h"
#include <phidgets/motor_encoder.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <functional>
#include <random>
#include <chrono>


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

    struct Particle{
        double xPos;
        double yPos;
        double thetaPos;
        double weight;
        Particle (): xPos(0), yPos(0), thetaPos(0), weight(1) {}

    };

    //Particle::Particle(double x, double y, double theta, double weight): xPos(x), yPos(y), thetaPos(theta), weight(weight) {}




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
        encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &FilterPublisher::encoderCallbackLeft, this);
        encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &FilterPublisher::encoderCallbackRight, this);

        wheel_r = 0.04;
        base_d = 0.25;
        tick_per_rotation = 900;
        control_frequenzy = 10; //10 hz
        dt = 1/control_frequenzy;

        k_D=1;
        k_V=1;
        k_W=1;

        int x_spread = 10;
        int y_spread = 10;
        int nr_particles = 100;

        initializeParticles(x_spread, y_spread, nr_particles);


    }


    void encoderCallbackLeft(const phidgets::motor_encoder::ConstPtr& msg){
        encoding_abs_new[0] = msg->count;


    }

    void encoderCallbackRight(const phidgets::motor_encoder::ConstPtr& msg){
        encoding_abs_new[1] = -(msg->count);

    }

    void initializeParticles(int x_spead, int y_spread, int nr_particles){

        particles.resize(nr_particles);
        for (int i = 0; i<nr_particles; i++){
            particles[i].xPos = (double) i;
            particles[i].yPos = (double) i;
            particles[i].thetaPos = (double) i;
        }

        for (int i = 0; i<nr_particles; i++){
            ROS_INFO("Attributes for particle nr: [%d] -  [%f], [%f], [%f], [%f]\n",i , particles[i].xPos, particles[i].yPos, particles[i].thetaPos, particles[i].weight);
        }


    }

    void localize(){



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

        dist_D = std::normal_distribution<double>(0.0, pow((linear_v*dt*k_D), 2));
        dist_V = std::normal_distribution<double>(0.0, pow((linear_v*dt*k_V), 2));
        dist_W = std::normal_distribution<double>(0.0, pow((angular_w*dt*k_W), 2));


    }
    void sample_motion_model(Particle p){

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

    void measurement_model(Particle p){
        //Sample the measurements
    }

private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<double> encoding_delta;
    std::default_random_engine generator;
    std::normal_distribution<double> dist_D;
    std::normal_distribution<double> dist_V;
    std::normal_distribution<double> dist_W;
    std::vector<Particle> particles;

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
    ROS_INFO("Spin!");


    double frequency = 10;
    ros::init(argc, argv, "filter_publisher");

    FilterPublisher filter(frequency);


    ros::Rate loop_rate(frequency);

    int count = 0;
    while (filter.n.ok()){

        filter.localize();
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
