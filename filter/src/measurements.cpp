
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

#include <measurements.h>


pair<float, float> localToWorldCoordinates(float x_particle, float y_particle, float theta_particle, float lidar_x, float lidar_y, float lidar_orientation, float range_laser, float angle_laser)
{

    float x_world = x_particle +
                    cos(theta_particle) * lidar_x - sin(theta_particle) * lidar_y +
                    range_laser * cos(theta_particle + (angle_laser + lidar_orientation));
    float y_world = y_particle +
                    sin(theta_particle) * lidar_x + cos(theta_particle) * lidar_y +
                    range_laser * sin(theta_particle + (angle_laser + lidar_orientation));

    return pair<float, float>(x_world, y_world);
}

pair<float, float> particleToLidarConversion(float x_particle, float y_particle, float theta_particle, float lidar_x, float lidar_y)
{

    float x_world = x_particle +
                    cos(theta_particle) * lidar_x - sin(theta_particle) * lidar_y;
    float y_world = y_particle +
                    sin(theta_particle) * lidar_x + cos(theta_particle) * lidar_y;

    return pair<float, float>(x_world, y_world);
}

float distancesToRange(pair<float, float> wall_distances, pair<float, float> base_distances)
{

    float x1 = wall_distances.first;
    float y1 = wall_distances.second;

    float x0 = base_distances.first;
    float y0 = base_distances.second;

    float distance = sqrt(pow(x1 - x0, 2) + pow(y0 - y1, 2));

    return distance;
}

pair<int, int> getClosestWallCoordinates(vector<vector<unsigned char>> global_map, float cellSize, float angle, pair<int, int> base_coordinates)
{
    int x = base_coordinates.first;
    int y = base_coordinates.second;
    bool wallFound = false;

    float maxDistance = 2.0 / cellSize;

    int count = 0;
    while (!wallFound && count < maxDistance)
    {
        if (global_map[x][y] == 1)
        {
            wallFound = true;
        }
        else
        {
            x = round(x + cos(angle));
            y = round(y + sin(angle));
            count++;
        }
    }

    return pair<int, int>(x, y);
}

//MIGHT HAVE TO LOOK FOR INFINITES HERE IN CLOSESTWALLCOORDINATES
vector<pair<float, float>> calculateRealRange(LocalizationGlobalMap map, float translated_particle_x, float translated_particle_y, vector<pair<float, float>> laser_data, float lidar_orientation)
{
    pair<int, int> particle_coordinates = map.getCell(translated_particle_x, translated_particle_y);

    pair<float, float> particle_real_coordinates = map.getDistance(particle_coordinates.first, particle_coordinates.second);    

    float currentAngle = 0;

    vector<pair<float, float>> ranges;

    for(int i = 0; i < laser_data.size(); i++) {

        float distance_real = 0;        

        currentAngle = laser_data[i].first + lidar_orientation;
        
        pair<int, int> closestWall_coordinates = getClosestWallCoordinates(map.global_map, map.cellSize, currentAngle, particle_coordinates);
        
        pair<float, float> wall_real_coordinates = map.getDistance(closestWall_coordinates.first, closestWall_coordinates.second);

        distance_real = distancesToRange(wall_real_coordinates, particle_real_coordinates);

        pair<float, float> range = make_pair(distance_real, laser_data[i].second);

        ranges.push_back(range);
    }

    return ranges;
}

float calculateWeight(LocalizationGlobalMap map, float translated_particle_x, float translated_particle_y, vector<pair<float, float>> laser_data, float max_distance, float lidar_orientation)
{
    float weight = 0;

    float z_hit;
    float z_short;
    float z_max;
    float z_random;

    float sigma_hit;
    float lambda_short;

    vector<pair<float, float>> rangeWithTrueRange = calculateRealRange(map, translated_particle_x, translated_particle_y, laser_data, lidar_orientation);

    float q = 1;

    for (int r = 0; r < rangeWithTrueRange.size(); r++)
    {
        float prob_hit = 0;
        float prob_short = 0;
        float prob_max = 0;
        float prob_random = 0;

        float realRange = rangeWithTrueRange[r].first;
        float measuredRange = rangeWithTrueRange[r].second;

        float p = 0;

        // Calculate the hit probability
        if (0 <= measuredRange && measuredRange <= max_distance)
        {
            normal_distribution<float> distribution(realRange, sigma_hit);
            float prob = distribution(measuredRange);

            // CALCULATE ETA, FIND SOLUTION LATER
            float eta_hit = 0;

            prob_hit = prob * eta_hit;
        }

        // Calculate the short (unexpected objects) probability
        if (0 <= measuredRange && measuredRange <= realRange)
        {
            float eta_short = 1 / (1 - exp(-lambda_short * realRange));

            prob_short = eta_short * lambda_short * exp(-lambda_short * measuredRange);
        }

        // Calculate the max probability
        if (measuredRange > max_distance)
        {
            prob_max = 1;
        }

        // Calculate the random readings probability
        if (0 <= measuredRange && measuredRange < max_distance)
        {
            prob_random = 1 / max_distance;
        }

        p = z_hit * prob_hit + z_short * prob_short + z_max * prob_max + z_random * prob_random;

        q *= p;
    }

    weight = q;

    return weight;
}


void getParticlesWeight(vector<Particle> &particles, LocalizationGlobalMap map, vector<pair<float, float>> laser_data, float max_distance)
{
    float weight = 0;

    float lidar_x = 0.1;
    float lidar_y = 0;
    float lidar_orientation = M_PI/2;

    for (int p = 0; p < particles.size(); p++)
    {
        //pair<float, float> new_particle_coordinates = localToWorldCoordinates(x, y, theta, lidar_x, lidar_y, lidar_orientation, laser_data.first, laser_data.second);
        
        pair<float, float> new_particle_center = particleToLidarConversion(particles[p].xPos, particles[p].yPos, particles[p].thetaPos, lidar_x, lidar_y);

        weight = calculateWeight(map, new_particle_center.first, new_particle_center.second, laser_data, max_distance, lidar_orientation);
        particles[p].weight = weight;
    }

}
