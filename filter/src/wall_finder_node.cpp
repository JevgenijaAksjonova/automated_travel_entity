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
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"

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
    ros::Publisher wall_publisher;
    ros::Publisher wall_array_publisher;



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
    float _angular_velocity;
    int _wasTurning;
    struct Outlier{
        float xPos;
        float yPos;
        float probability;
        int index;
    };

    struct Wall{
        float xStart;
        float yStart;
        float xEnd;
        float yEnd;
        float xCenter;
        float yCenter;
        float angle;
        float length;
        int nrAgreeingPoints;
        bool published;
    };
    vector<Wall> _wallsFound;




    //LocalizationGlobalMap map;

    WallFinder()
    {
        n = ros::NodeHandle("~");
        int nr_measurements = 8;
        _xPos = 0;
        _yPos = 0;
        _thetaPos = 0;
        _begunMoving = false;
        _wasTurning = 0;


        if(!n.getParam("/wall_finder/nr_measurements",nr_measurements)){
            ROS_ERROR("wf failed to detect parameter1");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/outlier_treshold",OUTLIER_THRESHOLD)){
            ROS_ERROR("wf failed to detect parameter2 ");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/MAX_DISTANCE_LIDAR",MAX_DISTANCE_LIDAR)){
            ROS_ERROR("wf failed to detect parameter3");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/MIN_OUTLIERS_IN_ROW",MIN_OUTLIERS_IN_ROW)){
            ROS_ERROR("wf failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/MAX_INDEX_DISTANCE_BETWEEN_OUTLIERS_IN_ROW",MAX_INDEX_DISTANCE_BETWEEN_OUTLIERS_IN_ROW)){
            ROS_ERROR("wf failed to detect parameter 5");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/MAX_EUCLIDEAN_DISTANCE_BETWEEN_OUTLIERS",MAX_EUCLIDEAN_DISTANCE_BETWEEN_OUTLIERS)){
            ROS_ERROR("wf failed to detect parameter 6");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/MIN_POINTS",MIN_POINTS)){
            ROS_ERROR("wf failed to detect parameter 7");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/MAX_DISTANCE_TO_OUTLIER",MAX_DISTANCE_TO_OUTLIER)){
            ROS_ERROR("wf failed to detect parameter 8");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/wall_finder/ANGULAR_VELOCITY_TRESHOLD",ANGULAR_VELOCITY_TRESHOLD)){
            ROS_ERROR("wf failed to detect parameter 9");
            exit(EXIT_FAILURE);
        }


        ROS_INFO("Running wall finder with parameters:");
        ROS_INFO("Number of measurements: [%d]", nr_measurements);
        ROS_INFO("Outlier treshold: [%f]", OUTLIER_THRESHOLD);

        lidar_subscriber = n.subscribe("/scan", 1, &WallFinder::lidarCallback, this);
        position_subscriber = n.subscribe("/filter", 1, &WallFinder::positionCallback, this );
        outlier_publisher = n.advertise<visualization_msgs::MarkerArray>("/visual_outliers", 1);
        wall_publisher= n.advertise<visualization_msgs::MarkerArray>("/wall_finder_walls_visual", 1);
        wall_array_publisher= n.advertise<std_msgs::Float32MultiArray>("/wall_finder_walls_array", 100);

        _nr_measurements = nr_measurements;

    }

    void addKnownWalls(LocalizationGlobalMap &map){
        for(int i = 0; i < map.walls.size(); i++){
            Wall w = createWall(map.walls[i][0], map.walls[i][1], map.walls[i][2], map.walls[i][3], 1000, true);
            _wallsFound.push_back(w);

        }
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
        _angular_velocity = abs(msg->twist.twist.angular.z);
    }
    
    void lookForWalls(LocalizationGlobalMap map){
        if(_angular_velocity < ANGULAR_VELOCITY_TRESHOLD){
            ROS_INFO("angular_v ok, %f", _angular_velocity);
            if(_wasTurning >0){
                _wasTurning --;
            }else{
            vector<pair<float, float>> measurements = mapMeasurementsToAngles();
            getOutliers(map, measurements);
            }
        }else{
            ROS_INFO("NOT RUNNING DUE TO TOO HIGH ANGULAR VELOCITY, %f", _angular_velocity);
            _wasTurning = 3;
        }
    }

    vector<pair<float, float>> mapMeasurementsToAngles()
    {
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
            
            if (range > MAX_DISTANCE_LIDAR)
            {
                range = MAX_DISTANCE_LIDAR;
            }

            
            std::pair<float, float> angle_measurement(angle, range);
            sampled_measurements.push_back(angle_measurement);
            i = i + step_size;
        }
        return sampled_measurements;
    }


    void getOutliers(LocalizationGlobalMap map, vector<pair<float, float>> sampled_measurements){

        pair<float, float> translatedCenters = particleToLidarConversion(_xPos, _yPos, _thetaPos, LIDAR_X, LIDAR_Y);
        float translated_x = translatedCenters.first;
        float translated_y = translatedCenters.second;

        vector<Outlier> outlierCandidates;
        vector<Outlier> confirmedOutliers;

        float sumOfProbs = 0;


        if (sampled_measurements.size() > 0);
        {
            for(int i = 0; i<sampled_measurements.size(); i++){                
                pair<float, float> measurement = sampled_measurements[i];

                vector<pair<float, float>> temp_measurements;
                temp_measurements.push_back(measurement);
                float prob = calculateWeight(map, translated_x, translated_y, temp_measurements, MAX_DISTANCE_LIDAR, _thetaPos);
                Outlier o = addOutlierMeasurement(translated_x, translated_y, _thetaPos, measurement.first, measurement.second, prob, i);
                //ROS_INFO("outlier %d", i);
                //ROS_INFO("location: x [%f] y [%f] probability [%f] ", o.xPos, o.yPos, o.probability);
                outlierCandidates.push_back(o);
                sumOfProbs += prob;
            }

            float averageProb = sumOfProbs/sampled_measurements.size();
            for(int i = 0; i<outlierCandidates.size(); i++){
                Outlier candidate = outlierCandidates[i];
                float distanceToCandidate = sqrt(pow((candidate.xPos - _xPos),2) + pow((candidate.yPos - _yPos),2));
                //ROS_INFO("average prob [%f] outlier prob [%f]",averageProb,candidate.probability);
                if(candidate.probability < averageProb * OUTLIER_THRESHOLD && distanceToCandidate < MAX_DISTANCE_TO_OUTLIER){
                    confirmedOutliers.push_back(candidate);
                }
            }

            int i = 0;
            vector<Outlier> rowCandidate;
            vector<vector<Outlier>> outlierRows;
            while(i < confirmedOutliers.size()){
                int outLierIndex = confirmedOutliers[i].index;
                    int j = i+1;
                    if(j < confirmedOutliers.size() && confirmedOutliers[j].index < outLierIndex + MAX_INDEX_DISTANCE_BETWEEN_OUTLIERS_IN_ROW && calculateDistanceBetweenOutliers(confirmedOutliers[i], confirmedOutliers[j])<MAX_EUCLIDEAN_DISTANCE_BETWEEN_OUTLIERS){
                        rowCandidate.push_back(confirmedOutliers[i]);
                    }else{
                        if(rowCandidate.size() > MIN_OUTLIERS_IN_ROW){
                            outlierRows.push_back(rowCandidate);
                        }
                        else{
                        }
                        rowCandidate.clear();

                    }
                    i++;
            }


            if(outlierRows.size() > 0) {
                ROS_INFO("--------nr of walls found this iteration: %lu", outlierRows.size());
                for(int i = 0; i < outlierRows.size(); i++){
                    float rowStartX = outlierRows[i][0].xPos;
                    float rowStartY = outlierRows[i][0].yPos;

                    float rowEndX = outlierRows[i].back().xPos;
                    float rowEndY = outlierRows[i].back().yPos;
                    Wall w = createWall(rowStartX, rowStartY, rowEndX, rowEndY, outlierRows[i].size(), false);
                    addWall(w, map);
                }
            }
            forgetWalls();
            if(_wallsFound.size() > 0){
                publish_rviz_walls();
                publish_array_walls();
            }
            publish_rviz_outliers(confirmedOutliers);

        
        }
    }
    void addWall(Wall w, LocalizationGlobalMap &map){
        bool wallIsNew = true;
        bool wallIsInsideMap = true;
        bool robotInsideWall = false;
        int i = 0;
        ROS_INFO("Trying to add wall [%f] [%f] [%f] [%f]", w.xStart, w.yStart, w.xEnd, w.yEnd);
        while(wallIsNew && !robotInsideWall && i < _wallsFound.size()){
            wallIsNew = checkIfNewWall(w, _wallsFound[i], i);
           robotInsideWall = checkIfRobotInsideWall(w);
            i++;
        }
        if(w.xStart < map.xMin || w.xStart > map.xMax || w.yStart < map.yMin || w.yStart > map.yMax){
            wallIsInsideMap = false;
        }
        if(w.xEnd < map.xMin || w.xEnd > map.xMax || w.yEnd < map.yMin || w.yEnd > map.yMax){
            wallIsInsideMap = false;
        }
        if(wallIsNew && wallIsInsideMap && !robotInsideWall){
            ROS_INFO("*****************Found new wall!***********************");
            _wallsFound.push_back(w);
        }
    }
    void forgetWalls(){

        vector<Wall>::iterator iter = _wallsFound.begin();
            while (iter != _wallsFound.end() && !(*iter).published){
                if((*iter).nrAgreeingPoints < 1){
                    iter = _wallsFound.erase(iter);
                }
                else
                {
                    (*iter).nrAgreeingPoints --;
                    ++iter;
                }
            }
    }

    float calculateLinePointDistance(float &x, float &y, float &x1, float &y1, float &x2, float &y2 ){

        float A = x - x1;
        float B = y - y1;
        float C = x2 - x1;
        float D = y2 - y1;

        float dot = A * C + B * D;
        float len_sq = C * C + D * D;
        float param = -1;
        if (len_sq != 0) //in case of 0 length line
            param = dot / len_sq;

        float xx, yy;

        if (param < 0) {
            xx = x1;
            yy = y1;
        }
        else if (param > 1) {
            xx = x2;
            yy = y2;
        }
        else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        float dx = x - xx;
        float dy = y - yy;
        return sqrt(dx * dx + dy * dy);
    }

    bool checkIfNewWall(Wall wNew, Wall wOld, int i){

        float x1 = wOld.xStart;
        float x2 = wOld.xEnd;
        float y1 = wOld.yStart;
        float y2 = wOld.yEnd;
        float x = wNew.xCenter;
        float y = wNew.yCenter;

        float centerDistance = calculateLinePointDistance(x, y, x1, y1, x2, y2);
        float angleDifference =  M_PI - abs(abs(wNew.angle - wOld.angle) - M_PI); 
        ROS_INFO("Comparing to wall %d,  [%f] [%f] [%f] [%f]", i, wOld.xStart, wOld.yStart, wOld.xEnd, wOld.yEnd);
        ROS_INFO("Distance to old wall %d is %f", i, centerDistance);
        if(centerDistance > 0.05 && angleDifference > M_PI/5){
            return true;
        }
        if(centerDistance > 0.15){
            return true;
        }
        ROS_INFO("Wall is the same angle difference : %f", angleDifference);
        ROS_INFO("Previoud points %d",_wallsFound[i].nrAgreeingPoints);
        if(wNew.length >wOld.length){ //Assume longer is better.
            ROS_INFO("Updating wall %d", i);
            ROS_INFO("Published before = %d", _wallsFound[i].published);
            _wallsFound[i] = wNew;
            _wallsFound[i].nrAgreeingPoints += wOld.nrAgreeingPoints;
            _wallsFound[i].published = wOld.published;
            ROS_INFO("Published = %d", _wallsFound[i].published);
        }else{
            _wallsFound[i].nrAgreeingPoints += wNew.nrAgreeingPoints;
        }
        // ROS_INFO("updated to %d",_wallsFound[i].nrAgreeingPoints);
        return false;

    }

    bool checkIfRobotInsideWall(Wall w){
    	float distance = calculateLinePointDistance(_xPos, _yPos, w.xStart, w.yStart, w.xEnd, w.yEnd);
    	if(distance < 0.1){
    		ROS_INFO("Robot was inside wall [%f] [%f] [%f] [%f]", w.xStart, w.yStart, w.xEnd, w.yEnd);
    		return true;
    	}
    	return false;

    }

    Wall createWall(float xStart, float yStart, float xEnd, float yEnd, int nrPoints, bool published){

        float centre_x = (xStart + xEnd) / 2;
        float centre_y = (yStart + yEnd) / 2;

        float rotation = atan2((yEnd - yStart), (xEnd - xStart));
        //rotation = rotation % M_PI;
        float length = sqrt(pow(yEnd - yStart, 2) + pow(xEnd - xStart, 2));

        Wall w;
        w.xStart = xStart;
        w.yStart = yStart;
        w.xEnd = xEnd;
        w.yEnd = yEnd;
        w.xCenter = centre_x;
        w.yCenter = centre_y;
        w.angle = rotation;
        w.length = length;
        w.nrAgreeingPoints = nrPoints;
        w.published = published;

        return w;

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

        float weight = 0.02;

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

    void publish_rviz_walls(){

        visualization_msgs::MarkerArray found_walls;
        for(int i = 0; i < _wallsFound.size(); i++){
            Wall w = _wallsFound[i];
            //ROS_INFO("Wall %d: center x %f y %f nrPoints %d", i, w.xCenter, w.yCenter, w.nrAgreeingPoints);
            if(w.nrAgreeingPoints > MIN_POINTS && !w.published){

                visualization_msgs::Marker wall;

                wall.header.frame_id = "/odom";
                wall.header.stamp = ros::Time::now();

                wall.ns = "global_wall";
                wall.type = visualization_msgs::Marker::CUBE;
                wall.action = visualization_msgs::Marker::ADD;

                wall.pose.position.z = 0.1;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                //marker.scale.x = 1.0;
                wall.scale.y = 0.01;
                wall.scale.z = 0.3;

                // Set the color -- be sure to set alpha to something non-zero!
                wall.color.r = 1.0f;
                wall.color.g = 0.0f;
                wall.color.b = 0.0f;
                wall.color.a = 1.0;


                wall.pose.position.x = w.xCenter;
                wall.pose.position.y = w.yCenter;

                tf::Quaternion q;
                q.setRPY(0.0, 0.0, w.angle);
                tf::quaternionTFToMsg(q, wall.pose.orientation);

                wall.scale.x = w.length;

                wall.id = i;
                found_walls.markers.push_back(wall);
            }
        }

        wall_publisher.publish(found_walls);
    }

    void publish_array_walls(){
        for(int i = 0; i < _wallsFound.size(); i++){
            Wall w = _wallsFound[i];
            //ROS_INFO("Wall %d: center x %f y %f nrPoints %d", i, w.xCenter, w.yCenter, w.nrAgreeingPoints);
            std_msgs::Float32MultiArray array;
            array.data.clear();
            if(w.nrAgreeingPoints > MIN_POINTS && !w.published){
                array.data.push_back(w.xStart);
                array.data.push_back(w.yStart);
                array.data.push_back(w.xEnd);
                array.data.push_back(w.yEnd);
                wall_array_publisher.publish(array);
                _wallsFound[i].published = true;
                ROS_INFO("*********************Published wall %d [%f] [%f] [%f] [%f]******************'", i, w.xStart, w.yStart, w.xEnd, w.yEnd);


            }
        }

    }

    float calculateDistanceBetweenOutliers(Outlier &first, Outlier &second){
        return sqrt(pow((second.xPos - first.xPos),2) + pow((second.yPos - first.yPos),2));
    }

  private:

    std::vector<float> prob_meas;
    float OUTLIER_THRESHOLD;
    float LIDAR_X = -0.03;
    float LIDAR_Y = 0;
    float MAX_DISTANCE_LIDAR;
    int MIN_OUTLIERS_IN_ROW;
    int MAX_INDEX_DISTANCE_BETWEEN_OUTLIERS_IN_ROW;
    float MAX_EUCLIDEAN_DISTANCE_BETWEEN_OUTLIERS;
    int MIN_POINTS;
    float MAX_DISTANCE_TO_OUTLIER;
    float ANGULAR_VELOCITY_TRESHOLD;

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
    wf.addKnownWalls(map);

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
