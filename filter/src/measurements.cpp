
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
vector<pair<float, float>> calculateRealRange(LocalizationGlobalMap map, Particle particle, vector<pair<float, float>> laser_data)
{

    float x = particle.xPos;
    float y = particle.yPos;
    float theta = particle.thetaPos;

    float lidar_x = 0.1;
    float lidar_y = 0;
    float lidar_orientation = M_PI/2;

    
    pair<int, int> particle_coordinates = map.getCell(x, y);

    pair<float, float> particle_real_coordinates = map.getDistance(particle_coordinates.first, particle_coordinates.second);    

    float currentAngle = 0;

    vector<pair<float, float>> ranges;

    for(int i = 0; i < laser_data.size(); i++) {

        float distance_real = 0;        

        currentAngle = laser_data[i].first;
        
        pair<int, int> closestWall_coordinates = getClosestWallCoordinates(map.global_map, map.cellSize, currentAngle, particle_coordinates);
        
        pair<float, float> wall_real_coordinates = map.getDistance(closestWall_coordinates.first, closestWall_coordinates.second);

        distance_real = distancesToRange(wall_real_coordinates, particle_real_coordinates);


        //pair<float, float> measurement_coordinates = localToWorldCoordinates(x, y, theta, lidar_x, lidar_y, lidar_orientation, laser_data.first, laser_data.second);    
                


        pair<float, float> range = make_pair(distance_real, laser_data[i].second);

        ranges.push_back(range);
    }

    return ranges;
}

float calculateWeight(LocalizationGlobalMap map, Particle particle, vector<pair<float, float>> laser_data, double max_distance)
{
    double weight = 0;

    double z_hit;
    double z_short;
    double z_max;
    double z_random;

    double sigma_hit;
    double lambda_short;

    vector<pair<double, double>> rangeWithTrueRange = calculateRealRange(map, particle, laser_data);

    double q = 1;

    for (int r = 0; r < rangeWithTrueRange.size(); r++)
    {
        double prob_hit = 0;
        double prob_short = 0;
        double prob_max = 0;
        double prob_random = 0;

        double realRange = rangeWithTrueRange[r].first;
        double measuredRange = rangeWithTrueRange[r].second;

        double p = 0;

        // Calculate the hit probability
        if (0 <= measuredRange && measuredRange <= max_distance)
        {
            normal_distribution<double> distribution(realRange, sigma_hit);
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

    for (int p = 0; p < particles.size(); p++)
    {
        weight = calculateWeight(map, particles[p], laser_data, max_distance);
        particles[p].weight = weight;
    }

}
