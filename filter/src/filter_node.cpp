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
#include <stdlib.h>
#include <pwd.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <localization_global_map.h>
#include <measurements.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;

class FilterPublisher
{
  public:
    ros::NodeHandle n;

    ros::Publisher filter_publisher;
    ros::Publisher particle_publisher;

    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    float pi;
    std::vector<float> dphi_dt;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber lidar_subscriber;
    std::vector<float> ranges;
    float angle_increment;
    float range_min;
    float range_max;
    int _nr_measurements;
    int _nr_random_particles;
    bool _using_random_particles;
    float _gaussian_particle_noise_spread;

    //LocalizationGlobalMap map;

    FilterPublisher(float frequency)
    {
        control_frequency = frequency;
        dt = 1/control_frequency;
        n = ros::NodeHandle("~");
        int nr_particles = 500;
        int nr_measurements = 8;
        int nr_random_particles = 10;
        float random_particle_spread = 0.1;
        float k_D = 0.5;
        float k_V = 0.5;
        float k_W = 0.5;
        bool using_random_particles = false;
        float gaussian_particle_noise_spread = 0.1;

        
        if(!n.getParam("/filter/particle_params/nr_particles",nr_particles)){
            ROS_ERROR("Filter failed to detect parameter 1");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/nr_measurements",nr_measurements)){
            ROS_ERROR("Filter failed to detect parameter 2");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/nr_random_particles",nr_random_particles)){
            ROS_ERROR("Filter failed to detect parameter 3");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/random_particle_spread",random_particle_spread)){
            ROS_ERROR("Filter failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/gaussian_particle_noise_spread",gaussian_particle_noise_spread)){
            ROS_ERROR("Filter failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }

        if(!n.getParam("/filter/particle_params/using_random_particles",using_random_particles)){
            ROS_ERROR("Filter failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/odom_noise/k_D",k_D)){
            ROS_ERROR("Filter failed to detect parameter 5");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/odom_noise/k_V",k_V)){
            ROS_ERROR("Filter failed to detect parameter 6");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/odom_noise/k_W",k_W)){
            ROS_ERROR("Filter failed to detect parameter 7");
            exit(EXIT_FAILURE);
        }

        _using_random_particles = using_random_particles;
        _gaussian_particle_noise_spread = gaussian_particle_noise_spread;

        ROS_INFO("Running filter with parameters:");
        ROS_INFO("Number of particles: [%d]", nr_particles);
        ROS_INFO("Number of measurements: [%d]", nr_measurements);
        ROS_INFO("Number of random particles: [%d]", nr_random_particles);
        ROS_INFO("Random particle spread: [%f]", random_particle_spread);
        ROS_INFO("Odom k_V: [%f]", k_V);
        ROS_INFO("Odom k_D: [%f]", k_D);
        ROS_INFO("Odom k_W: [%f]", k_W);
        pi = 3.1416;

        encoding_abs_prev = std::vector<int>(2, 0);
        encoding_abs_new = std::vector<int>(2, 0);
        encoding_delta = std::vector<float>(2, 0);
        dphi_dt = std::vector<float>(2, 0.0);

        //set up random
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);

        first_loop = true;

        filter_publisher = n.advertise<nav_msgs::Odometry>("/filter", 1);
        particle_publisher = n.advertise<visualization_msgs::MarkerArray>("/visual_particles", 1);
        encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &FilterPublisher::encoderCallbackLeft, this);
        encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &FilterPublisher::encoderCallbackRight, this);
        lidar_subscriber = n.subscribe("/scan", 1, &FilterPublisher::lidarCallback, this);

        _wheel_r = 0.04;
        _base_d = 0.2;
        tick_per_rotation = 900;

        _k_D = k_D;
        _k_V = k_V;
        _k_W = k_W;

        float start_x = 0.215;
        float start_y = 0.26;
        float spread_xy = 0.05;
        float start_theta = pi / 2;
        float spread_theta = pi / 40;
        srand(static_cast<unsigned>(time(0)));
        particle_randomness = std::normal_distribution<float>(0.0, random_particle_spread);
        particle_randomness2 = std::normal_distribution<float>(0.0, _gaussian_particle_noise_spread);
        _nr_measurements = nr_measurements;
        _nr_random_particles = nr_random_particles;

        initializeParticles(start_x, start_y, spread_xy, start_theta, spread_theta, nr_particles);
    }

    void encoderCallbackLeft(const phidgets::motor_encoder::ConstPtr &msg)
    {
        encoding_abs_new[0] = msg->count;
    }

    void encoderCallbackRight(const phidgets::motor_encoder::ConstPtr &msg)
    {
        encoding_abs_new[1] = -(msg->count);
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        ranges = msg->ranges;
        angle_increment = msg->angle_increment;

        range_min = msg->range_min;
        range_max = msg->range_max;
    }

    void initializeParticles(float start_x, float start_y, float spread_xy, float start_theta, float spread_theta, int nr_particles)
    {

        std::normal_distribution<float> dist_start_x = std::normal_distribution<float>(start_x, spread_xy);
        std::normal_distribution<float> dist_start_y = std::normal_distribution<float>(start_y, spread_xy);
        std::normal_distribution<float> dist_start_theta = std::normal_distribution<float>(start_theta, spread_theta);

        particles.resize(nr_particles);
        for (int i = 0; i < nr_particles; i++)
        {

            particles[i].xPos = dist_start_x(generator);
            particles[i].yPos = dist_start_y(generator);
            particles[i].thetaPos = dist_start_theta(generator);
            particles[i].weight = (float)1.0 / nr_particles;
        }
    }

    Particle localize(LocalizationGlobalMap map)
    {

        //Reset weights
        for (int m = 0; m < particles.size(); m++)
        {
            particles[m].weight = (float)1.0 / particles.size();
        }

        calculateVelocityAndNoise();

         if (linear_v != 0 || angular_w != 0)
         {
            //update according to odom
            for (int m = 0; m < particles.size(); m++)
            {
                sample_motion_model(particles[m]);
            }
            //update weights according to measurements

            measurement_model(map);

            float weight_sum = 0.0;
            for (int m = 0; m < particles.size(); m++)
            {
                weight_sum += particles[m].weight;
            }

            // Normalize weights here
            for (int i = 0; i < particles.size(); i++)
            {
                particles[i].weight = particles[i].weight / weight_sum;
            }

            if(_using_random_particles){
                resampleParticlesWithRandomParticles();
            }else{
                resampleParticlesWithGaussianNoise();

            }
        }


        return getPositionEstimation();
    }

    Particle getPositionEstimation(){
        float x_estimate = 0;
        float y_estimate = 0;
        float theta_estimate = 0;
        float sinPos;
        float cosPos;
        for (int m = 0; m < particles.size() - _nr_random_particles; m++)
        {
            x_estimate += particles[m].xPos;
            y_estimate += particles[m].yPos;
            sinPos +=  sin(particles[m].thetaPos);
            cosPos +=  cos(particles[m].thetaPos);

        }
        Particle p;
        p.xPos = x_estimate/(particles.size() - _nr_random_particles);
        p.yPos = y_estimate/(particles.size() - _nr_random_particles);
        p.thetaPos = atan2(sinPos, cosPos);

        return p;

    }

    void resampleParticlesWithRandomParticles()
    {
        float rand_num;
        float cumulativeProb;
        int j;
        std::vector<Particle> temp_vec;
        for (int i = 0; i < (particles.size() - _nr_random_particles); i++)
        {
            j = 0;
            cumulativeProb = particles[0].weight;
            rand_num = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX));

            while ((cumulativeProb < rand_num) && j<particles.size()-1)
            {
                j++;
                cumulativeProb += particles[j].weight;
            }

            temp_vec.push_back(particles[j]);
        }

        //add random particle
        int v1;
        particle_randomness(generator);
        for (int i = 0; i < _nr_random_particles; i++)
        {
            v1 = rand() % temp_vec.size();
            Particle p = temp_vec[v1];
            p.xPos += particle_randomness(generator);;
            p.yPos += particle_randomness(generator);;
            p.thetaPos += particle_randomness(generator);;
            temp_vec.push_back(p);
        }

        particles.swap(temp_vec);
    }

    // WORKS VERY BAD, WHY?
    void resampleParticlesWithGaussianNoise()
    {
        float rand_num;
        float cumulativeProb;
        int j;
        std::vector<Particle> temp_vec;
        for (int i = 0; i < (particles.size()); i++)
        {
            j = 0;
            cumulativeProb = particles[0].weight;
            rand_num = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX));

            while ((cumulativeProb < rand_num) && j<particles.size()-1)
            {
                j++;
                cumulativeProb += particles[j].weight;
            }

            temp_vec.push_back(particles[j]);
        }
        
        particles.swap(temp_vec);
        for(int i = 0; i<particles.size(); i++){
            particles[i].xPos += particle_randomness2(generator);
            particles[i].yPos += particle_randomness2(generator);
            particles[i].thetaPos += particle_randomness2(generator);
        }
    }

    void calculateVelocityAndNoise()
    {

        if (first_loop)
        {
            encoding_abs_prev[0] = encoding_abs_new[0];
            encoding_abs_prev[1] = encoding_abs_new[1];
        }
        first_loop = false;

        encoding_delta[0] = encoding_abs_new[0] - encoding_abs_prev[0];
        encoding_delta[1] = encoding_abs_new[1] - encoding_abs_prev[1];

        encoding_abs_prev[0] = encoding_abs_new[0];
        encoding_abs_prev[1] = encoding_abs_new[1];

        dphi_dt[0] = ((encoding_delta[0]) / (tick_per_rotation)*2 * pi) / dt;
        dphi_dt[1] = ((encoding_delta[1]) / (tick_per_rotation)*2 * pi) / dt;

        linear_v = (_wheel_r / 2) * (dphi_dt[1] + dphi_dt[0]);
        angular_w = (_wheel_r / _base_d) * (dphi_dt[1] - dphi_dt[0]);

        dist_D = std::normal_distribution<float>(0.0, pow((linear_v * dt*_k_D), 2));
        dist_V = std::normal_distribution<float>(0.0, pow((linear_v * dt*_k_V), 2));
        dist_W = std::normal_distribution<float>(0.0, pow((angular_w * dt*_k_W), 2));
    }

    void sample_motion_model(Particle &p)
    {
        float noise_D = dist_D(generator);
        float noise_V = dist_V(generator);
        float noise_W = dist_W(generator);

        p.xPos += (linear_v * dt + noise_D) * cos(p.thetaPos);
        p.yPos += (linear_v * dt + noise_D) * sin(p.thetaPos);
        p.thetaPos += (angular_w * dt + noise_W) + noise_V;

        if (p.thetaPos > pi)
        {
            p.thetaPos = p.thetaPos - 2 * pi;
        }
        if (p.thetaPos < -pi)
        {
            p.thetaPos = p.thetaPos + 2 * pi;
        }

        // One alternative would be to se TF Matrix to translate p to lidar_link
    }

    void measurement_model(LocalizationGlobalMap map)
    {
        //Sample the measurements
        float lidar_x = -0.03;
        float lidar_y = 0.0;
        int step_size = (ranges.size() / _nr_measurements);
        std::vector<pair<float, float>> sampled_measurements;
        float start_angle = -pi/2;
        float max_distance = 3.0;
        float range;
        float angle = 0;

        int i = 0;

        while (i < ranges.size())
        {
            angle = (i * angle_increment) + start_angle;
            range = ranges[i];

            int j = 1;
            while(std::isinf(range)) {
                range = ranges[i + j];
                angle += angle_increment;
                j++;
            }
            
            if (range > max_distance)
            {
                range = max_distance;
            }

            
            std::pair<float, float> angle_measurement(angle, range);
            sampled_measurements.push_back(angle_measurement);
            i = i + step_size;
        }

        if (ranges.size() > 0)
        {

            getParticlesWeight(particles, map, sampled_measurements, max_distance, lidar_x, lidar_y);
        }
    }

    void publishPosition(Particle ml_pos, Particle ml_pos_prev)
    {
        ros::Time current_time = ros::Time::now();
        float theta = atan2(sin(ml_pos.thetaPos)+sin(ml_pos_prev.thetaPos), cos(ml_pos_prev.thetaPos) +cos(ml_pos.thetaPos));
        float x = (ml_pos.xPos + ml_pos_prev.xPos)/2;
        float y = (ml_pos.yPos + ml_pos_prev.yPos)/2;


        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        // Publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        //set the velocity

        float vx = linear_v * cos(theta);
        float vy = linear_v * sin(theta);
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = angular_w;

        filter_publisher.publish(odom_msg);

    }

    void collect_measurements(std::vector<std::pair<float, float>> &sampled_measurements, LocalizationGlobalMap map)
    {
        int nr_measurements_used = 8;
        int step_size = (ranges.size() / nr_measurements_used);
        float start_angle = -pi/2;
        float max_distance = 3.0;
        float range;
        float angle = 0;

        int i = 0;

        while (i < ranges.size())
        {
            angle = (i * angle_increment) + start_angle;
            range = ranges[i];

            int j = 1;
            while(std::isinf(range)) {
                range = ranges[i + j];
                angle += angle_increment;
                j++;
            }
            
            if (range > max_distance)
            {
                range = max_distance;
            }

            
            ROS_INFO("Range: [%f]", range);
            ROS_INFO("Angle: [%f]", angle);
            std::pair<float, float> angle_measurement(angle, range);
            sampled_measurements.push_back(angle_measurement);
            i = i + step_size;
        }

        ROS_INFO("sampled measurements  [%lu]", sampled_measurements.size());

        if (sampled_measurements.size() > 3000)
        {
            run_calibrations(map, sampled_measurements);
        }
    }

    void run_calibrations(LocalizationGlobalMap map, std::vector<std::pair<float, float>> &sampled_measurements)
    {
        float max_distance = 3.0;
        float pos_x = 0.205;
        float pos_y = 0.335;
        float theta = pi / 2;

        std::pair<float, float> xy = particleToLidarConversion(pos_x, pos_y, theta, -0.03, 0.0);

        float z_hit = 0.25;
        float z_short = 0.25;
        float z_max = 0.25;
        float z_random = 0.25;
        float sigma_hit = 0.2;
        float lambda_short = 0.5;
        while (true)
        {
            ROS_INFO("Before calculate");
            calculateIntrinsicParameters(map, sampled_measurements, max_distance, xy.first, xy.second, theta, z_hit, z_short, z_max, z_random, sigma_hit, lambda_short);
            ROS_INFO("Parameters found zhit:[%f] zshort:[%f] zmax:[%f], zradom:[%f], sigmahit[%f], lambdashort:[%f] ", z_hit, z_short, z_max, z_random, sigma_hit, lambda_short);
        }
    }

    void publish_rviz_particles()
    {
        ros::Time current_time = ros::Time::now();
        
        visualization_msgs::MarkerArray all_particles;
        visualization_msgs::Marker particle;

        particle.header.stamp = current_time;
        particle.header.frame_id = "/odom";

        particle.ns = "all_particles";
        particle.type = visualization_msgs::Marker::CUBE;
        particle.action = visualization_msgs::Marker::ADD;

        particle.pose.position.z = 0.05;

        // Set the color -- be sure to set alpha to something non-zero!
        particle.color.r = 0.0f;
        particle.color.g = 0.0f;
        particle.color.b = 1.0f;
        particle.color.a = 1.0;

        float weight = 0.01;

        int id = 0;
        for (int i = 0; i < particles.size(); i++)
        {

            particle.pose.position.x = particles[i].xPos;
            particle.pose.position.y = particles[i].yPos;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            particle.scale.y = weight;
            particle.scale.x = weight;
            particle.scale.z = weight;

            particle.id = id;
            id++;

           all_particles.markers.push_back(particle);
        }

        particle_publisher.publish(all_particles);
    }


  private:
    std::vector<int> encoding_abs_prev;
    std::vector<int> encoding_abs_new;
    std::vector<float> encoding_delta;
    std::default_random_engine generator;
    std::normal_distribution<float> dist_D;
    std::normal_distribution<float> dist_V;
    std::normal_distribution<float> dist_W;
    std::normal_distribution<float> particle_randomness;
    std::normal_distribution<float> particle_randomness2;
    std::vector<Particle> particles;

    float _wheel_r;
    float _base_d;
    int tick_per_rotation = 900;
    float control_frequency;
    float dt;
    bool first_loop;
    float linear_v;
    float angular_w;

    float _k_D;
    float _k_V;
    float _k_W;
};

int main(int argc, char **argv)
{

    ROS_INFO("Spin!");

    float frequency = 10;
    struct passwd *pw = getpwuid(getuid());
    std::string homePath(pw->pw_dir);

    std::string _filename_map = homePath+"/catkin_ws/src/automated_travel_entity/filter/maps/lab_maze_2017.txt";
    float cellSize = 0.01;

    ros::init(argc, argv, "filter_publisher");


    FilterPublisher filter(frequency);

    LocalizationGlobalMap map(_filename_map, cellSize);

    ros::Rate loop_rate(frequency);

    Particle most_likely_position;
    Particle most_likely_position_prev;
    most_likely_position_prev.xPos = 0.0;
    most_likely_position_prev.yPos = 0.0;
    most_likely_position_prev.thetaPos = 0.0;
    std::vector<std::pair<float, float>> sampled_measurements;



    int count = 0;
    while (filter.n.ok())
    {


        most_likely_position = filter.localize(map);
        most_likely_position_prev = most_likely_position;
        filter.publishPosition(most_likely_position, most_likely_position_prev);
        filter.publish_rviz_particles();
        //filter.collect_measurements(sampled_measurements, map);
        most_likely_position_prev = most_likely_position;
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
