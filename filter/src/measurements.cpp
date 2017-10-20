#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

using namespace std;

pair<float, float> localToWorldCoordinates(float x_particle, float y_particle, float theta_particle, float lidar_x, float lidar_y, float lidar_orientation, float range_laser, float angle_laser) {

    float x_world = x_particle +
            cos(theta_particle) * lidar_x - sin(theta_particle) * lidar_y +
            range_laser * cos(theta_particle + (angle_laser + lidar_orientation));
    float y_world = y_particle +
            sin(theta_particle) * lidar_x + cos(theta_particle) * lidar_y +
            range_laser * sin(theta_particle + (angle_laser + lidar_orientation));

    return pair<float, float>(x_world, y_world);
}

float distancesToRange(pair<float, float> wall_distances, pair<float, float> base_distances) {

    float x1 = wall_distances.first;
    float y1 = wall_distances.second;

    float x0 = base_distances.first;
    float y0 = base_distances.second;

    float distance = sqrt(pow(x1 - x0, 2) + pow(y0 - y1, 2));

    return distance;
}

pair<int, int> getClosestWallCoordinates(vector<vector<unsigned char> > global_map, float cellSize, float angle, int x_base, int y_base) {
    int x = x_base;
    int y = y_base;
    bool wallFound = false;

    float maxDistance = 2.0/cellSize;

    int count = 0;
    while(!wallFound && count < maxDistance) {
        if(global_map[x][y] == 1) {
            wallFound = true;
        } else {
            x = round(x + cos(angle));
            y = round(y + sin(angle));
            count++;
        }
    }

    return pair<int, int>(x, y);
}


/*
std::vector<std::vector<int>> createLocalMap(std::vector<float> ranges, float angle_increment, float x_particle, float y_particle, float theta_particle, std::vector<std::vector<int>> local_map)
{
    float current_angle = M_PI;
    float x_offset = 0.10;
    float y_offset = 0;
    float theta_offset = M_PI;

    int x_max;
    int y_max;
    int x_min;
    int y_min;

    // std::vector<std::vector<int>> global_map;
    // Should be in calling function
    // memset(local_map, 0, sizeof(global_map));


    for(int i = 0; i < ranges.size(); i++) {
        if(!isinf(ranges[i])) {
            pair<float, float> world_coordinates = localToWorldCoordinates(x_particle, y_particle, theta_particle, x_offset, y_offset, theta_offset, ranges[i], current_angle);

            local_map[x_world_coordinate][y_world_coordinate] = 1;
        }
        current_angle -= angle_increment;
    }


    return local_map;
}

*/

/*
float particleWeight(std::vector<std::vector<int>> local_map, std::vector<std::vector<int>> global_map) {

    int average_map = 0;
    int map_sum = 0;
    int N = 0;

    for(int row = 0; row < local_map[0].length; row++) {
        for(int col = 0; col < local_map.length; col++) {
            map_sum += local_map[row][col] + global_map[row][col];

            if(local_map[row][col] == global_map[row][col]) {
                N++;
            }
        }
    }
    if(N != 0) {
        average_map = (1 / (2*N)) * map_sum;
    }


}
*/

/*
std::vector<float> calculate_weights(std::vector<float> ranges, std::vector<Particle>, float angle_increment) {
    tf::TransformListener listener;

    tf::StampedTransform transform;

    try{
        listener.lookupTransform("/laser", "/odom",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    float weight = calculate_weight(ranges, angle_increment, x_particle, y_particle, theta_particle);

}
*/
