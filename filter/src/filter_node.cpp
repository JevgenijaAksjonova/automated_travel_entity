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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <project_msgs/stop.h>


#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

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
    ros::Publisher stuck_position_publisher;
    ros::Publisher stuck_wall_publisher;
    ros::Publisher stuck_publisher;

    ros::Subscriber encoder_subscriber_left;
    ros::Subscriber encoder_subscriber_right;
    ros::Subscriber initalPose_subscriber;
    ros::Subscriber navigation_speed_subscriber;
    ros::Subscriber addedWall_subscriber;
    ros::Subscriber motherWantsToMove_subscriber;

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
    bool _intitialPoseReceived;
    float _start_x;
    float _start_y;
    float _start_theta;
    int _nr_particles;
    float linear_v;
    float angular_w;
    bool RUN_WHILE_STANDING_STILL;
    float _navigation_linear_speed;
    float _navigation_angular_speed;
    ros::Time _laserTime;
    bool _motherWantsToMove;


    LocalizationGlobalMap map;

    //LocalizationGlobalMap map;

    FilterPublisher(float frequency, LocalizationGlobalMap newMap)
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
         _intitialPoseReceived = false;
         map = newMap;
         _navigation_linear_speed = 0;
         _navigation_angular_speed = 0;
         _laserTime = ros::Time::now();

        
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
        if(!n.getParam("/filter/general/RUN_WHILE_STANDING_STILL",RUN_WHILE_STANDING_STILL)){
            ROS_ERROR("Filter failed to detect parameter 8");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/general/STUCK_TRESHOLD_SPEED",STUCK_TRESHOLD_SPEED)){
            ROS_ERROR("Filter failed to detect parameter 9");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/general/STUCK_TRESHOLD_DISTANCE",STUCK_TRESHOLD_DISTANCE)){
            ROS_ERROR("Filter failed to detect parameter 10");
            exit(EXIT_FAILURE);
        }

        _using_random_particles = using_random_particles;
        _gaussian_particle_noise_spread = gaussian_particle_noise_spread;
        _nr_particles = nr_particles;

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
        stuck_publisher = n.advertise<project_msgs::stop>("navigation/obstacles",100);
        stuck_position_publisher = n.advertise<std_msgs::Float32MultiArray>("/stuck_position", 100);

        encoder_subscriber_left = n.subscribe("/motorcontrol/encoder/left", 1, &FilterPublisher::encoderCallbackLeft, this);
        encoder_subscriber_right = n.subscribe("/motorcontrol/encoder/right", 1, &FilterPublisher::encoderCallbackRight, this);
        lidar_subscriber = n.subscribe("/scan", 1, &FilterPublisher::lidarCallback, this);
        initalPose_subscriber = n.subscribe("/initialpose", 1 ,&FilterPublisher::initialPoseCallback, this);
        navigation_speed_subscriber = n.subscribe("/motor_controller/twist", 1, &FilterPublisher::navigation_speed_encoder, this);
        motherWantsToMove_subscriber = n.subscribe("/mother/moving", 1, &FilterPublisher::motherWantsToMoveCallback, this);

        //addedWall_subscriber = n.subscribe("/wall_finder_walls_array", 1, &FilterPublisher::addedWallCallback, this);



        _wheel_r = 0.04;
        _base_d = 0.2;
        tick_per_rotation = 900;

        _k_D = k_D;
        _k_V = k_V;
        _k_W = k_W;

        srand(static_cast<unsigned>(time(0)));
        particle_randomness = std::normal_distribution<float>(0.0, random_particle_spread);
        particle_randomness2 = std::normal_distribution<float>(0.0, _gaussian_particle_noise_spread);
        _nr_measurements = nr_measurements;
        _nr_random_particles = nr_random_particles;
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
        _laserTime = msg->header.stamp;
        ranges = msg->ranges;
        angle_increment = msg->angle_increment;

        range_min = msg->range_min;
        range_max = msg->range_max;
    }

    void navigation_speed_encoder(const geometry_msgs::Twist::ConstPtr &msg){
        _navigation_angular_speed = msg->linear.x; 
        _navigation_linear_speed = msg->angular.z;

    }


    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
        _start_x = msg->pose.pose.position.x;
        _start_y = msg->pose.pose.position.y;
        ROS_INFO("Initial pose recieved start_x [%f], start_y [%f] ", _start_x, _start_y);
        tf::Pose pose;
        tf::poseMsgToTF(msg->pose.pose, pose);
        _start_theta = tf::getYaw(pose.getRotation());
        _intitialPoseReceived = true;
        ROS_INFO("Initial pose theta [%f] ", _start_theta);
    }

    void addedWallCallback(const std_msgs::Float32MultiArray::ConstPtr& array){
	    vector<double> wall;
	    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
	        wall.push_back(*it);
	    }
	    if(wall.size() != 4){
	        ROS_INFO("WALL HAS WERID DIMENSIONS %lu", wall.size());
	    }else{
	    	ROS_INFO("ADDING WALL TO FILTER MAP");
	    	//map.walls.push_back(wall);
	    }
    }

    void motherWantsToMoveCallback(const std_msgs::Bool::ConstPtr &msg){
        _motherWantsToMove = msg->data;
    }

    void initializeParticles()
    {
        ROS_INFO("intitializing particles!");
        float spread_xy = 0.05;
        float spread_theta = pi / 40;

        std::normal_distribution<float> dist_start_x = std::normal_distribution<float>(_start_x, spread_xy);
        std::normal_distribution<float> dist_start_y = std::normal_distribution<float>(_start_y, spread_xy);
        std::normal_distribution<float> dist_start_theta = std::normal_distribution<float>(_start_theta, spread_theta);

        particles.resize(_nr_particles);

        for (int i = 0; i < _nr_particles; i++)
        {

            particles[i].xPos = dist_start_x(generator);
            particles[i].yPos = dist_start_y(generator);
            particles[i].thetaPos = dist_start_theta(generator);
            particles[i].weight = (float)1.0 / _nr_particles;
        }
}

    Particle localize()
    {

        //Reset weights
        for (int m = 0; m < particles.size(); m++)
        {
            particles[m].weight = (float)1.0 / particles.size();
        }

        calculateVelocityAndNoise();

        if ( (linear_v != 0 || angular_w != 0) || RUN_WHILE_STANDING_STILL )
        {

            //update according to odom
            for (int m = 0; m < particles.size(); m++)
            {
                sample_motion_model(particles[m]);
            }
            //update weights according to measurements

            measurement_model();

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

    void measurement_model()
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

    void publishPosition(Particle ml_pos)
    {
        ros::Time current_time = ros::Time::now();
        float theta = ml_pos.thetaPos;
        float x = ml_pos.xPos;
        float y = ml_pos.yPos;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        geometry_msgs::TransformStamped odom_trans;
        //odom_trans.header.stamp = _laserTime;
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
        odom_msg.header.stamp = _laserTime;
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

    /*
    void collect_measurements(std::vector<std::pair<float, float>> &sampled_measurements)
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
            run_calibrations(sampled_measurements);
        }
    }

    void run_calibrations(std::vector<std::pair<float, float>> &sampled_measurements)
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
    */

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

    void checkIfStuck(Particle &ml_pos, Particle &ml_pos_prev, vector<float> linearV_vec, vector<bool> motherWantsToMove_vec){

        float dx = ml_pos.xPos - ml_pos_prev.xPos;
        float dy = ml_pos.yPos - ml_pos_prev.yPos;
        float diff = ml_pos_prev.thetaPos - atan2(dy, dx);
        float distance = sqrt(pow(ml_pos.xPos - ml_pos_prev.xPos, 2) + pow(ml_pos.yPos - ml_pos_prev.yPos, 2));
        float linear_v_calc = cos(diff)*distance/dt;
        
        float averageLinearV = 0;
        for(int i = 0; i < linearV_vec.size(); i++){
            averageLinearV += linearV_vec[i];
        }
        averageLinearV = averageLinearV/linearV_vec.size();

        int i = 0;
        bool motherWantedToMove = true;
        while(i < motherWantsToMove_vec.size() && motherWantedToMove ){
            motherWantedToMove = motherWantsToMove_vec[i];
            i++;
        }
        motherWantedToMove = false;
        ROS_INFO("Average linear V [%f], distance moved [%f]", averageLinearV, distance);
        if((averageLinearV > STUCK_TRESHOLD_SPEED || motherWantedToMove) && distance < STUCK_TRESHOLD_DISTANCE){
            ROS_INFO("THINK WE ARE STUCK");
            ROS_INFO("average Linear V [%f], distance moved [%f]", averageLinearV, distance);
            ROS_INFO("Mother wanted to move %d", motherWantedToMove);
            publish_stuck(ml_pos);
        }


    }

    void publish_stuck(Particle &ml_pos){

        //Publish emergency stop
        project_msgs::stop msg;
        msg.stamp = ros::Time::now();
        msg.stop = true;
        msg.reason = 5;
        stuck_publisher.publish(msg);

        //Publish where we got stuck
        std_msgs::Float32MultiArray array;
        array.data.clear();
        array.data.push_back(ml_pos.xPos);
        array.data.push_back(ml_pos.yPos);
        array.data.push_back(ml_pos.thetaPos);
        stuck_position_publisher.publish(array);
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
    

    float _k_D;
    float _k_V;
    float _k_W;
    float STUCK_TRESHOLD_SPEED;
    float STUCK_TRESHOLD_DISTANCE;
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
    ROS_INFO("Creating map");
    LocalizationGlobalMap map(_filename_map, cellSize);
    ROS_INFO("Map created!");


    FilterPublisher filter(frequency, map);


    ros::Rate loop_rate(frequency);

    Particle most_likely_position;
    std::vector<std::pair<float, float>> sampled_measurements;


    filter._start_x = 0.215;
    filter._start_y = 0.230;
    filter._start_theta = M_PI/2;
    filter.initializeParticles();
    Particle most_likely_position_prev;
    most_likely_position_prev.xPos = 0;
    most_likely_position_prev.yPos = 0;
    most_likely_position_prev.thetaPos = 0;

    vector<float> linear_v_vec;
    vector<bool> motherWantsToMove_vec;
    int count = 0;
    while (filter.n.ok())
    {


        if(filter._intitialPoseReceived){
            filter.initializeParticles();
            filter._intitialPoseReceived = false;
            ROS_INFO("Ready to run!");
        }


        most_likely_position = filter.localize();
        filter.publishPosition(most_likely_position);

        filter.publish_rviz_particles();

        linear_v_vec.push_back(filter._navigation_linear_speed);
        motherWantsToMove_vec.push_back(filter._motherWantsToMove);

        if(count % 100 == 0){
            filter.checkIfStuck(most_likely_position, most_likely_position_prev, linear_v_vec, motherWantsToMove_vec);
            linear_v_vec.clear();
            motherWantsToMove_vec.clear();
            most_likely_position_prev = most_likely_position;

        }

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
