#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>


std::vector<std::vector<int>> createLocalMap(std::vector<float> ranges, float angle_increment, float x_particle, float y_particle, float theta_particle, std::vector<std::vector<int>> local_map)
{
    float current_angle = M_PI;
    float x_offset = 0.10;
    float y_offset = 0;
    float theta_offset = M_PI;

    int x_max;
    int y_max;





// std::vector<std::vector<int>> global_map;
// Should be in calling function
// memset(local_map, 0, sizeof(global_map));


    for(int i = 0; i < ranges.size(); i++) {
        if(!isinf(ranges[i])) {
            float x_world = x_particle +
                            cos(theta_particle) * x_offset - sin(theta_particle) * y_offset +
                            ranges[i] * cos(theta_particle + (current_angle + theta_offset));
            float y_world = y_particle +
                            sin(theta_particle) * x_offset + cos(theta_particle) * y_offset +
                            ranges[i] * sin(theta_particle + (current_angle + theta_offset));

            int x_world_coordinate = int(x_world * 100);
            int y_world_coordinate = int(y_world * 100);

            local_map[x_world_coordinate][y_world_coordinate] = 1;
        }
        current_angle -= angle_increment;
    }


    return local_map;
}

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
