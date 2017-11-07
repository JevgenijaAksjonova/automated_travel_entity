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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <localization_global_map.h>
#include <measurements.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

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

    //LocalizationGlobalMap map;

    FilterPublisher(int frequency)
    {
        control_frequency = frequency;
        n = ros::NodeHandle("~");
        pi = 3.1416;

        encoding_abs_prev = std::vector<int>(2, 0);
        encoding_abs_new = std::vector<int>(2, 0);
        encoding_delta = std::vector<float>(2, 0);
        dphi_dt = std::vector<float>(2, 0.0);

        //set up random
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        generator = std::default_random_engine(seed);

        first_loop = true;

        filter_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
        particle_publisher = n.advertise<visualization_msgs::MarkerArray>("/visual_particles", 1);
        encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &FilterPublisher::encoderCallbackLeft, this);
        encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &FilterPublisher::encoderCallbackRight, this);
        lidar_subscriber = n.subscribe("/scan", 1, &FilterPublisher::lidarCallback, this);

        wheel_r = 0.04;
        base_d = 0.25;
        tick_per_rotation = 900;
        control_frequenzy = 10; //10 hz
        dt = 1 / control_frequenzy;

        k_D = 1;
        k_V = 1;
        k_W = 1;

        float start_xy = 0.2;
        float spread_xy = 0.02;
        float start_theta = pi / 2;
        float spread_theta = pi / 40;
        int nr_particles = 100;

        initializeParticles(start_xy, spread_xy, start_theta, spread_theta, nr_particles);
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

    void initializeParticles(float start_xy, float spread_xy, float start_theta, float spread_theta, int nr_particles)
    {

        std::normal_distribution<float> dist_start_xy = std::normal_distribution<float>(start_xy, spread_xy);
        std::normal_distribution<float> dist_start_theta = std::normal_distribution<float>(start_theta, spread_theta);

        particles.resize(nr_particles);
        for (int i = 0; i < nr_particles; i++)
        {

            particles[i].xPos = dist_start_xy(generator);
            particles[i].yPos = dist_start_xy(generator);
            particles[i].thetaPos = dist_start_theta(generator);
            particles[i].weight = (float)1.0 / nr_particles;
        }

        // for (int i = 0; i<nr_particles; i++){
        //     ROS_INFO("Attributes for particle nr: [%d] -  [%f], [%f], [%f], [%f]\n",i , particles[i].xPos, particles[i].yPos, particles[i].thetaPos, particles[i].weight);
        // }
    }

    Particle localize(LocalizationGlobalMap map)
    {

        //Reset weights
        for (int m = 0; m < particles.size(); m++)
        {
            particles[m].weight = (float)1.0 / particles.size();
        }

        calculateVelocityAndNoise();

        //update according to odom
        for (int m = 0; m < particles.size(); m++)
        {
            sample_motion_model(particles[m]);
        }
        //update weights according to measurements
        ROS_INFO("Before measure model");
        measurement_model(map);
        ROS_INFO("After measure model");

        //sample particles with replacement
        float weight_sum = 0.0;
        Particle most_likely_position;
        most_likely_position.weight = 0.0;
        srand(static_cast<unsigned>(time(0)));
        for (int m = 0; m < particles.size(); m++)
        {
            weight_sum += particles[m].weight;
            ROS_INFO("Weight of particle [%d] is [%f]", m, particles[m].weight);
            if (particles[m].weight > most_likely_position.weight)
            {
                most_likely_position = particles[m];
                //ROS_INFO("New most likely position!: -  [%f], [%f], [%f], [%f]\n", most_likely_position.xPos, most_likely_position.yPos, most_likely_position.thetaPos, most_likely_position.weight);
            }
        }

        float rand_num;
        float cumulativeProb;
        int j;
        std::vector<Particle> temp_vec;

        //if (linear_v != 0 || angular_w != 0)
        //{
            for (int i = 0; i < particles.size(); i++)
            {
                j = 0;
                cumulativeProb = 0.0;
                rand_num = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / weight_sum));

                while (cumulativeProb < rand_num)
                {
                    cumulativeProb += particles[j].weight;
                    j++;
                }

                temp_vec.push_back(particles[j]);
            }
            particles = temp_vec;
        //}
        return most_likely_position;
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

        linear_v = (wheel_r / 2) * (dphi_dt[1] + dphi_dt[0]);
        angular_w = (wheel_r / base_d) * (dphi_dt[1] - dphi_dt[0]);

        dist_D = std::normal_distribution<float>(0.0, pow((linear_v * dt * k_D), 2));
        dist_V = std::normal_distribution<float>(0.0, pow((linear_v * dt * k_V), 2));
        dist_W = std::normal_distribution<float>(0.0, pow((angular_w * dt * k_W), 2));
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
        int nr_measurements_used = 4;
        int step_size = (ranges.size() / nr_measurements_used);
        std::vector<pair<float, float>> sampled_measurements;
        float angle = 0.0;
        float max_distance = 3.0;
        float range;

        int i = 0;

        while (i < ranges.size())
        {
            angle = -(i * angle_increment);
            range = ranges[i];
            if (std::isinf(range) || range > max_distance)
            {
                range = max_distance;
            }
            std::pair<float, float> angle_measurement(angle, range);
            sampled_measurements.push_back(angle_measurement);
            i = i + step_size;
        }

        if (ranges.size() > 0)
        {
            ROS_INFO("Readings have come");

            getParticlesWeight(particles, map, sampled_measurements, max_distance);
            //exit(EXIT_SUCCESS);
        }
        //update particle weights
    }

    void publishPosition(Particle ml_pos)
    {
        ros::Time current_time = ros::Time::now();

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(ml_pos.thetaPos);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = ml_pos.xPos;
        odom_trans.transform.translation.y = ml_pos.yPos;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        // Publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = ml_pos.xPos;
        odom_msg.pose.pose.position.y = ml_pos.yPos;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        //set the velocity

        float vx = linear_v * cos(ml_pos.thetaPos);
        float vy = linear_v * sin(ml_pos.thetaPos);
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = angular_w;

        filter_publisher.publish(odom_msg);

        ROS_INFO("new Position x:[%f] y:[%f] theta:[%f] ", ml_pos.xPos, ml_pos.yPos, ml_pos.thetaPos);
    }

    void collect_measurements(std::vector<std::pair<float, float>> &sampled_measurements, LocalizationGlobalMap map)
    {
        int nr_measurements_used = 8;
        int step_size = (ranges.size() / nr_measurements_used);
        float angle = 0;
        float max_distance = 3.0;
        float range;

        int i = 0;
        while (i < ranges.size())
        {
            angle = -(i * angle_increment);
            range = ranges[i];

            if (std::isinf(range) || range > max_distance)
            {
                range = max_distance;
            }

            std::pair<float, float> angle_measurement(angle, range);
            sampled_measurements.push_back(angle_measurement);
            i = i + step_size;
        }

        ROS_INFO("sampled measurements  [%lu]", sampled_measurements.size());

        if (sampled_measurements.size() > 4000)
        {
            run_calibrations(map, sampled_measurements);
        }

        /*
        vector<pair<float, float>> test_laser;
        test_laser.push_back(make_pair(0, 0.10));
        test_laser.push_back(make_pair(-pi/2, 0.20));
        test_laser.push_back(make_pair(-pi, 0.30));
        test_laser.push_back(make_pair(-3*pi/2, 0.40));
        std::pair<float, float> xy =  particleToLidarConversion(0.2, 0.2, pi/2, 0.095, 0.0);

        ROS_INFO("X_WORLD: %f, Y_WORLD: %f", xy.first, xy.second);

        vector<pair<float, float>> test_data = calculateRealRange(map, xy.first, xy.second, test_laser, pi/2);

        ROS_INFO("M, [%f]: R, [%f]", test_data[0].first, test_data[0].second);
        ROS_INFO("M, [%f]: R, [%f]", test_data[1].first, test_data[1].second);
        ROS_INFO("M, [%f]: R, [%f]", test_data[2].first, test_data[2].second);
        ROS_INFO("M, [%f]: R, [%f]", test_data[3].first, test_data[3].second);
        */
    }

    void run_calibrations(LocalizationGlobalMap map, std::vector<std::pair<float, float>> &sampled_measurements)
    {
        float max_distance = 3.0;
        float pos_x = 0.23;
        float pos_y = 0.2;
        float theta = pi / 2;

        std::pair<float, float> xy = particleToLidarConversion(pos_x, pos_y, theta, 0.095, 0.0);
        float lidar_orientation = pi / 2;
        float z_hit = 0.5;
        float z_short = 0.5;
        float z_max = 0.5;
        float z_random = 0.5;
        float sigma_hit = 0.2;
        float lambda_short = 0.5;
        while (true)
        {
            ROS_INFO("Before calculate");
            calculateIntrinsicParameters(map, sampled_measurements, max_distance, xy.first, xy.second, lidar_orientation, z_hit, z_short, z_max, z_random, sigma_hit, lambda_short);
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
    
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        //marker.scale.x = 1.0;
        particle.scale.y = 0.01;
        particle.scale.x = 0.01;
        particle.scale.z = 0.01;
    
        // Set the color -- be sure to set alpha to something non-zero!
        particle.color.r = 0.0f;
        particle.color.g = 1.0f;
        particle.color.b = 0.0f;
        particle.color.a = 1.0;

        int id = 0;
        for(int i = 0; i < particles.size(); i++) {

            particle.pose.position.x = particles[i].xPos;
            particle.pose.position.y = particles[i].yPos;     
            
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
    std::vector<Particle> particles;

    float wheel_r = 0.04;
    float base_d = 0.25;
    int tick_per_rotation = 900;
    float control_frequenzy = 10; //10 hz
    float dt = 1 / control_frequenzy;
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

    std::string _filename_map = "/home/rikko/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
    float cellSize = 0.01;

    ros::init(argc, argv, "filter_publisher");

    FilterPublisher filter(frequency);

    LocalizationGlobalMap map(_filename_map, cellSize);

    ros::Rate loop_rate(frequency);

    Particle most_likely_position;
    std::vector<std::pair<float, float>> sampled_measurements;

    int count = 0;
    while (filter.n.ok())
    {

        most_likely_position = filter.localize(map);
        filter.publishPosition(most_likely_position);
        filter.publish_rviz_particles();            
        //filter.collect_measurements(sampled_measurements, map);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
