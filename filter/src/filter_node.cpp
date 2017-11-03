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
#include <cstdlib>

#include "localization_global_map.h"
#include "measurements.h"


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

class FilterPublisher{
public:
    ros::NodeHandle n;
    ros::Publisher filter_publisher;
    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    float xpos;
    float ypos;
    float theta;
    float pi;
    std::vector<float> dphi_dt;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber lidar_subscriber;
    std::vector<float> ranges;
    float angle_increment;
    float range_min;
    float range_max;

    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);



    FilterPublisher(int frequency){
        control_frequency = frequency;
        n = ros::NodeHandle("~");
        pi = 3.1416;
        xpos = 0;
        ypos = 0;
        theta = 0;


        encoding_abs_prev = std::vector<int>(2,0);
        encoding_abs_new = std::vector<int>(2,0);
        encoding_delta = std::vector<float>(2,0);
        dphi_dt = std::vector<float>(2, 0.0);

        //set up random
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);

        first_loop = true;

        filter_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
        encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &FilterPublisher::encoderCallbackLeft, this);
        encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &FilterPublisher::encoderCallbackRight, this);
        lidar_subscriber = n.subscribe("/scan", 1, &FilterPublisher::lidarCallback, this);

        wheel_r = 0.04;
        base_d = 0.25;
        tick_per_rotation = 900;
        control_frequenzy = 10; //10 hz
        dt = 1/control_frequenzy;

        k_D=1;
        k_V=1;
        k_W=1;

        float start_xy = 5.0;
        float spread_xy = 2.0;
        float start_theta = 0.0;
        float spread_theta = pi/8;
        int nr_particles = 200;

        initializeParticles(start_xy, spread_xy, start_theta, spread_theta, nr_particles);


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


    void initializeParticles(float start_xy, float spread_xy, float start_theta, float spread_theta, int nr_particles){

        std::normal_distribution<float> dist_start_xy = std::normal_distribution<float>(start_xy, spread_xy);
        std::normal_distribution<float> dist_start_theta = std::normal_distribution<float>(start_theta, spread_theta);



        particles.resize(nr_particles);
        for (int i = 0; i<nr_particles; i++){
            
            particles[i].xPos = dist_start_xy(generator);
            particles[i].yPos = dist_start_xy(generator);
            particles[i].thetaPos = dist_start_theta(generator);
            particles[i].weight = (float) 1.0/nr_particles;
        }

        for (int i = 0; i<nr_particles; i++){
            ROS_INFO("Attributes for particle nr: [%d] -  [%f], [%f], [%f], [%f]\n",i , particles[i].xPos, particles[i].yPos, particles[i].thetaPos, particles[i].weight);
        }


    }

    void localize(){

        //Reset weights
        for (int m = 0; m < particles.size(); m++){
            particles[m].weight = (float) 1.0/particles.size();
        }
        
        calculateVelocityAndNoise();

        //update according to odom
        for (int m = 0; m < particles.size(); m++){
            sample_motion_model(particles[m]);
        }
        //update weights according to measurements
        measurement_model();

        //sample particles with replacement
        float weight_sum = 0.0;
        srand (static_cast <unsigned> (time(0)));
        for (int m = 0; m < particles.size(); m++){
            weight_sum += particles[m].weight;
        }

        float rand_num;
        float cumulativeProb = 0.0;
        int j;
        std::vector<Particle> temp_vec;
        
        for (int i = 0; i < particles.size(); i++){
            j = 0;
            rand_num= static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/weight_sum));

            while (cumulativeProb<rand_num){
                cumulativeProb += particles[j].weight;
                j++;
            }

            temp_vec.push_back(particles[j]);
    }
    particles.swap(temp_vec);
    



    }


    void calculateVelocityAndNoise(){

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

        dist_D = std::normal_distribution<float>(0.0, pow((linear_v*dt*k_D), 2));
        dist_V = std::normal_distribution<float>(0.0, pow((linear_v*dt*k_V), 2));
        dist_W = std::normal_distribution<float>(0.0, pow((angular_w*dt*k_W), 2));


    }
    void sample_motion_model(Particle &p){

        float noise_D = dist_D(generator);
        float noise_V = dist_V(generator);
        float noise_W = dist_W(generator);

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

    void measurement_model(){
        //Sample the measurements
        int nr_measurements_used = 4;
        int step_size = ranges.size()/nr_measurements_used;
        std::vector<pair<float, float>> sampled_measurements;
        float angle = 0.0;

        for(int i = 0; i = i + step_size; i<ranges.size()){
            angle = i*angle_increment;
            std::pair <float,float> angle_measurement (angle, ranges[i]);
            sampled_measurements.push_back(angle_measurement);
        }

        //update particle weights
        particlesWeight(particles, sampled_measurements);


        return (float) 1.0/particles.size();
    }

private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<float> encoding_delta;
    std::default_random_engine generator;
    std::normal_distribution<float> dist_D;
    std::normal_distribution<float> dist_V;
    std::normal_distribution<float> dist_W;
    std::vector<Particle> particles;

    float wheel_r = 0.04;
    float base_d = 0.25;
    int tick_per_rotation = 900;
    float control_frequenzy = 10; //10 hz
    float dt = 1/control_frequenzy;
    int control_frequency;
    bool first_loop;
    float linear_v;
    float angular_w;

    float k_D;
    float k_V;
    float k_W;


};


int main(int argc, char **argv)
{
    ROS_INFO("Spin!");


    float frequency = 10;
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
