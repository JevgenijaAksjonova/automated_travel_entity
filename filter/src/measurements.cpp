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

float calculateWeight(Particle particle, vector<float> laser_ranges, double max_distance) {
    double weight = 0;



    double z_hit;
    double z_short;
    double z_max;
    double z_random;

    double sigma_hit;
    double lambda_short;

    vector<pair<double, double>> rangeWithTrueRange = calculateTrueRange(particle, laser_ranges);

    double realRange = rangeWithTrueRange.first;
    double measuredRange = rangeWithTrueRange.second;

    double q = 1;

    for(int r = 0; r < rangeWithTrueRange.length; r++) {
        double prob_hit = 0;
        double prob_short = 0;
        double prob_max = 0;
        double prob_random = 0;

        double p = 0;

        // Calculate the hit probability
        if(0 <= measuredRange && measuredRange <= max_distance) {
            normal_distribution<double> distribution(realRange, sigma_hit);
            float prob = distribution(measuredRange);

            // CALCULATE ETA, FIND SOLUTION LATER
            float eta = 0;

            prob_hit = prob * eta;
        }

        // Calculate the short (unexpected objects) probability




        // Calculate the max probability


        // Calculate the random readings probability




        q *= p;
    }

}


vector<Particle> particlesWeight(vector<Particle> particles, vector<float> laser_ranges, float max_distance) {
    float weight = 0;

    for(int p = 0; p < particles.length; p++) {
        weight = calculateWeight(particles[i], laser_ranges, max_distance);
        particles[p].weight = weight;
    }


    return particles;
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
