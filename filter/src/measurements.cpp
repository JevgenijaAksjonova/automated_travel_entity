
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>
#include <random>
#include <functional>
#include <chrono>
#include <cmath>


#include <measurements.h>

pair<float, float> particleToLidarConversion(float x_particle, float y_particle, float theta_particle, float lidar_x, float lidar_y)
{

    float x_world = x_particle +
                    cos(theta_particle) * lidar_x - sin(theta_particle) * lidar_y;
    float y_world = y_particle +
                    sin(theta_particle) * lidar_x + cos(theta_particle) * lidar_y;

    return pair<float, float>(x_world, y_world);
}


vector<pair<float, float>> calculateRealRange(LocalizationGlobalMap map, float translated_particle_x, float translated_particle_y, vector<pair<float, float>> laser_data, float particle_theta)
{

    float currentAngle = 0;

    vector<pair<float, float>> ranges;
    
    // laser_data[0].first = -3.14/2;
    // laser_data[1].first = 0;
    // laser_data[2].first = 3.14/2;
    // laser_data[3].first = 3*3.14/2;

    // translated_particle_x = 0.205;
    // translated_particle_y = 0.305;
    // particle_theta = 3.14/2;
    //ROS_INFO("------------");

    for (int i = 0; i < laser_data.size(); i++)
    {

        currentAngle = laser_data[i].first + particle_theta;
        float rangeFromMap = map.getLineIntersection(translated_particle_x, translated_particle_y, currentAngle);

        pair<float, float> range = make_pair(rangeFromMap, laser_data[i].second);

        //ROS_INFO("RANGE ON MAP: %f", rangeFromMap);

        ranges.push_back(range);
    }

    //exit(EXIT_SUCCESS);    

    return ranges;
}

float get_dist_value(float mean, float sigma2, float number){
    float exponent = exp(((-1.0/2.0)*pow((number-mean), 2))/sigma2);
    float base = 1.0/sqrt(2.0*3.14*sigma2);
    return base*exponent;
}

float calculateWeight(LocalizationGlobalMap map, float translated_particle_x, float translated_particle_y, vector<pair<float, float>> laser_data, float max_distance, float particle_theta)
{
    if(translated_particle_x <0 || translated_particle_x > 2.4 || translated_particle_y < 0 || translated_particle_y > 2.409){
        return 0;
    }
    float weight = 0;

    float z_hit = 0.719204;
    float z_short = 0.080313;
    float z_max = 0.000000;
    float z_random = 0.200482;

    //float sigma_hit = 0.012170;
    float sigma_hit = 0.03;

    float lambda_short = 4.776181;
    

    vector<pair<float, float>> rangeWithTrueRange = calculateRealRange(map, translated_particle_x, translated_particle_y, laser_data, particle_theta);

    float q = 1;

    for (int r = 0; r < rangeWithTrueRange.size(); r++)
    {

        float prob_hit = 0;
        float prob_short = 0;
        float prob_max = 0;
        float prob_random = 0;

        float p = 0;

        float mapRange = rangeWithTrueRange[r].first;
        float laserRange = rangeWithTrueRange[r].second;
        if(mapRange > 3){
            return 0;
        }

        //ROS_INFO("mapRange [%f] laserRange [%f]", mapRange, laserRange);


        // Calculate the hit probability
        if (0 <= laserRange && laserRange <= max_distance)
        {
            
            float prob = get_dist_value(mapRange, pow(sigma_hit,2), laserRange);
            float eta_hit = 0.0;
            for(float i = 0; i < max_distance; i += 0.1){
                eta_hit += get_dist_value(mapRange, pow(sigma_hit,2), i);
            }

            prob_hit = prob / eta_hit;
        }

        // Calculate the short (unexpected objects) probability
        if (0 <= laserRange && laserRange <= mapRange)
        {
            float eta_short = 1 / (1 - exp(-lambda_short * mapRange));

            prob_short = eta_short * lambda_short * exp(-lambda_short * mapRange);
        }

        // Calculate the max probability
        if (laserRange >= max_distance)
        {
            prob_max = 1;
        }

        // Calculate the random readings probability
        if (0 <= laserRange && laserRange < max_distance)
        {
            prob_random = 1 / max_distance;
        }


        p = z_hit * prob_hit + z_short * prob_short + z_max * prob_max + z_random * prob_random;
        //ROS_INFO("p [%f]", p);

        q *= p;

    }

    weight = q;
    

    return weight;
}

void getParticlesWeight(vector<Particle> &particles, LocalizationGlobalMap map, vector<pair<float, float>> laser_data, float max_distance, float lidar_x, float lidar_y)
{
    float weight = 0;

    float x_map_max_distance = 2.4;
    float y_map_max_distance = 2.4;

    for (int p = 0; p < particles.size(); p++)
    {
        pair<float, float> new_particle_center = particleToLidarConversion(particles[p].xPos, particles[p].yPos, particles[p].thetaPos, lidar_x, lidar_y);


        if(new_particle_center.first < x_map_max_distance && new_particle_center.first > 0 && new_particle_center.second < y_map_max_distance && new_particle_center.second > 0) {
            weight = calculateWeight(map, new_particle_center.first, new_particle_center.second, laser_data, max_distance, particles[p].thetaPos); 
            
            
        } else {
            weight = 0.0;
        }


        particles[p].weight = weight;
    }
}

void calculateIntrinsicParameters(LocalizationGlobalMap map, vector<pair<float, float>> measurements, float max_distance, float pos_x, float pos_y, float theta, float &z_hit, float &z_short, float &z_max, float &z_random, float &sigma_hit, float &lambda_short)
{

    float e_hit = 0;
    float e_short = 0;
    float e_max = 0;
    float e_random = 0;

    float sigma_parameter = 0;
    float lambda_parameter = 0;

    vector<pair<float, float>> rangeWithTrueRange = calculateRealRange(map, pos_x, pos_y, measurements, theta);


    for (int r = 0; r < rangeWithTrueRange.size(); r++)
    {


        float prob_hit = 0;
        float prob_short = 0;
        float prob_max = 0;
        float prob_random = 0;

        float eta = 0;

        // REAL RANGE == THE RANGE FROM THE SENSOR
        // MEASURED RANGE == THE CALCULATED RANGE FROM THE POSITION AND MAP
        float measuredRange = rangeWithTrueRange[r].first;
        float realRange = rangeWithTrueRange[r].second;

        // Calculate the hit probability
        if (0 <= realRange && realRange <= max_distance)
        {
            //normal_distribution<float> distribution(realRange, sigma_hit);
            
            float prob = get_dist_value(measuredRange, pow(sigma_hit,2), realRange);
            float eta_hit = 0.0;
            for(float i = 0; i < max_distance; i += 0.1){
                eta_hit += get_dist_value(measuredRange, pow(sigma_hit,2), i);
            }

            // CALCULATE ETA, FIND SOLUTION LATER

            prob_hit = prob / eta_hit;
        }

        // Calculate the short (unexpected objects) probability
        if (0 <= realRange && realRange <= measuredRange)
        {
            float eta_short = 1 / (1 - exp(-lambda_short * measuredRange));

            prob_short = eta_short * lambda_short * exp(-lambda_short * measuredRange);
        }

        // Calculate the max probability
        if (realRange >= max_distance)
        {
            prob_max = 1;
        }

        // Calculate the random readings probability
        if (0 <= realRange && realRange < max_distance)
        {
            prob_random = 1 / max_distance;
        }

        eta = 1 / (prob_hit + prob_short + prob_max + prob_random);

       
        e_hit += eta * prob_hit;
        e_short += eta * prob_short;
        e_max += eta * prob_max;
        e_random += eta * prob_random;

        sigma_parameter += eta * prob_hit * pow(realRange - measuredRange, 2);

        lambda_parameter += eta * prob_short * realRange;
    }

    float Z_magnitude = rangeWithTrueRange.size();

    z_hit = e_hit / Z_magnitude;
    z_short = e_short / Z_magnitude;
    z_max = e_max / Z_magnitude;
    z_random = e_random / Z_magnitude;

    sigma_hit = sqrt((1 / e_hit) * sigma_parameter);

    lambda_short = e_short / lambda_parameter;

    ROS_INFO("IN FUNCITON: Parameters found zhit:[%f] zshort:[%f] zmax:[%f], zradom:[%f], sigmahit[%f], lambdashort:[%f] ", z_hit, z_short, z_max, z_random, sigma_hit, lambda_short);
}
