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
#include <sstream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <localization_global_map.h>
#include <measurements.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;

class WallFinder
{
  public:
    ros::NodeHandle n;

    ros::Subscriber lidar_subscriber;
    ros::Subscriber position_subscriber;
    ros::Publisher outlier_publisher;

    std::vector<float> ranges;
    float angle_increment;
    float range_min;
    float range_max;
    int _nr_measurements;
    float _xPos;
    float _yPos;
    float _thetaPos;
    std::vector<pair<float, float>> outliers;
    float _begunMoving;

    struct Outlier{
        float xPos;
        float yPos;
        float probability;
        int index;
    };



    //LocalizationGlobalMap map;

    WallFinder()
    {
        n = ros::NodeHandle("~");
        int nr_measurements = 8;
        _xPos = 0;
        _yPos = 0;
        _thetaPos = 0;
        _begunMoving = false;


        if(!n.getParam("/wall_finder/nr_measurements",nr_measurements)){
            ROS_ERROR("failed to detect parameter");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/outlier_treshold",OUTLIER_THRESHOLD)){
            ROS_ERROR("failed to detect parameter");
            exit(EXIT_FAILURE);
        }


        ROS_INFO("Running wall finder with parameters:");
        ROS_INFO("Number of measurements: [%d]", nr_measurements);
        ROS_INFO("Outlier treshold: [%f]", OUTLIER_THRESHOLD);

        lidar_subscriber = n.subscribe("/scan", 1, &WallFinder::lidarCallback, this);
        position_subscriber = n.subscribe("/filter", 1, &WallFinder::positionCallback, this );
        outlier_publisher = n.advertise<visualization_msgs::MarkerArray>("/visual_outliers", 1);


        _nr_measurements = nr_measurements;
    }


    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        ranges = msg->ranges;
        angle_increment = msg->angle_increment;

        range_min = msg->range_min;
        range_max = msg->range_max;
    }

    void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {

        _xPos = msg->pose.pose.position.x;//xStart - msg->pose.pose.position.y;
        _yPos = msg->pose.pose.position.y;//yStart + msg->pose.pose.position.x;

        geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
        _thetaPos = tf::getYaw(odom_quat);
        if(msg->twist.twist.linear.x > 0.0 || msg->twist.twist.linear.y > 0 ){
            _begunMoving = true;
        }
    }
    
    void lookForWalls(LocalizationGlobalMap map){
        ROS_INFO("Looking for walls!");
        ROS_INFO("at postition: x = [%f] y = [%f] theta = [%f]", _xPos, _yPos, _thetaPos);
        vector<pair<float, float>> measurements = mapMeasurementsToAngles();
        getOutliers(map, measurements);
    }

    vector<pair<float, float>> mapMeasurementsToAngles()
    {
        ROS_INFO("Sampling measurements");
        //Sample the measurements
        int step_size = (ranges.size() / _nr_measurements);
        std::vector<pair<float, float>> sampled_measurements;
        float start_angle = -M_PI/2;
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
            i = i +j;
            
            if (range > MAX_DISTANCE)
            {
                range = MAX_DISTANCE;
            }

            
            std::pair<float, float> angle_measurement(angle, range);
            sampled_measurements.push_back(angle_measurement);
            i = i + step_size;
        }
        return sampled_measurements;
    }


    void getOutliers(LocalizationGlobalMap map, vector<pair<float, float>> sampled_measurements){
        ROS_INFO("Getting outliers");

        pair<float, float> translatedCenters = particleToLidarConversion(_xPos, _yPos, _thetaPos, LIDAR_X, LIDAR_Y);
        float translated_x = translatedCenters.first;
        float translated_y = translatedCenters.second;

        vector<Outlier> outlierCandidates;
        vector<Outlier> confirmedOutliers;

        float sumOfProbs = 0;
        ROS_INFO("sampled_measurement size [%lu]", sampled_measurements.size());


        if (sampled_measurements.size() > 0);
        {
            for(int i = 0; i<sampled_measurements.size(); i++){                
                pair<float, float> measurement = sampled_measurements[i];

                vector<pair<float, float>> temp_measurements;
                temp_measurements.push_back(measurement);
                float prob = calculateWeight(map, translated_x, translated_y, temp_measurements, MAX_DISTANCE, _thetaPos);
                Outlier o = addOutlierMeasurement(translated_x, translated_y, _thetaPos, measurement.first, measurement.second, prob, i);
                //ROS_INFO("outlier %d", i);
                //ROS_INFO("location: x [%f] y [%f] probability [%f] ", o.xPos, o.yPos, o.probability);
                outlierCandidates.push_back(o);
                sumOfProbs += prob;
            }

            float averageProb = sumOfProbs/sampled_measurements.size();
            for(int i = 0; i<outlierCandidates.size(); i++){
                Outlier candidate = outlierCandidates[i];
                //ROS_INFO("average prob [%f] outlier prob [%f]",averageProb,candidate.probability);
                if(candidate.probability < averageProb * OUTLIER_THRESHOLD){
                    confirmedOutliers.push_back(candidate);
                }
            }

            ROS_INFO("looking for rows");
            int i = 0;
            vector<Outlier> rowCandidate;
            vector<vector<Outlier>> outlierRows;
            while(i < confirmedOutliers.size()){
                int outLierIndex = confirmedOutliers[i].index;
                    int j = i+1;
                    if(j < confirmedOutliers.size() && confirmedOutliers[j].index < outLierIndex + MAX_DISTANCE_BETWEEN_OUTLIERS_IN_ROW){
                        rowCandidate.push_back(confirmedOutliers[i]);    
                    }else{
                        if(rowCandidate.size() > MIN_OUTLIERS_IN_ROW){
                            outlierRows.push_back(rowCandidate);
                        }
                        else{
                            rowCandidate.clear();
                        }
                    }
                    i++;
            }
            ROS_INFO("nr of outlierRows = [%lu]", outlierRows.size());





            ROS_INFO("nr of outliers = [%lu]", confirmedOutliers.size());
            if(outlierRows.size() > 0) {
                ROS_INFO("5");
                for(int i = 0; i < outlierRows.size(); i++){
                    publish_rviz_outliers(outlierRows[i]);
                }
            }

        
        }
    }
        
    Outlier addOutlierMeasurement(float x, float y, float theta, float angle, float range, float prob, int i) {
            Outlier o;
            o.xPos = x + cos(theta + angle) * range;
            o.yPos = y + sin(theta + angle) * range;
            o.probability = prob;
            o.index = i;
            return o;

    }

    void publish_rviz_outliers(vector<Outlier> outliers)
    {
        ROS_INFO("publishing outliers");
        ros::Time current_time = ros::Time::now();
        
        visualization_msgs::MarkerArray all_outliers;
        visualization_msgs::Marker outlier;

        outlier.header.stamp = current_time;
        outlier.header.frame_id = "/odom";

        ros::Duration second(1, 0);

        outlier.lifetime = second;
        outlier.ns = "all_outliers";
        outlier.type = visualization_msgs::Marker::CUBE;
        outlier.action = visualization_msgs::Marker::ADD;

        outlier.pose.position.z = 0.05;

        // Set the color -- be sure to set alpha to something non-zero!
        outlier.color.r = 1.0f;
        outlier.color.g = 0.0f;
        outlier.color.b = 0.0f;
        outlier.color.a = 1.0;

        float weight = 0.01;

        int id = 0;
        for (int i = 0; i < outliers.size(); i++)
        {
            outlier.pose.position.x = outliers[i].xPos;
            outlier.pose.position.y = outliers[i].yPos;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            outlier.scale.y = weight;
            outlier.scale.x = weight;
            outlier.scale.z = weight;

            outlier.id = id;
            id++;

            all_outliers.markers.push_back(outlier);
        }

        outlier_publisher.publish(all_outliers);

        all_outliers.markers.clear();
    }

  private:

    std::vector<float> prob_meas;
    float OUTLIER_THRESHOLD;
    float LIDAR_X = -0.03;
    float LIDAR_Y = 0;
    float MAX_DISTANCE = 3;
    int MIN_OUTLIERS_IN_ROW = 5;
    int MAX_DISTANCE_BETWEEN_OUTLIERS_IN_ROW = 3;

};

int main(int argc, char **argv)
{

    ROS_INFO("Spin!");

    float frequency = 10;
    struct passwd *pw = getpwuid(getuid());
    std::string homePath(pw->pw_dir);

    std::string _filename_map = homePath+"/catkin_ws/src/automated_travel_entity/filter/maps/lab_maze_2017.txt";
    float cellSize = 0.01;

    ros::init(argc, argv, "wall_finder_publisher");


    WallFinder wf;

    LocalizationGlobalMap map(_filename_map, cellSize);

    ros::Rate loop_rate(frequency);



    int count = 0;
    while (wf.n.ok())
    {
        if(wf._begunMoving == true){
            wf.lookForWalls(map);

        }
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
