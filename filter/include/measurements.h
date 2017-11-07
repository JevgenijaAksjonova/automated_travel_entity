#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

#ifndef _MEASUREMENTS_INCLUDED
#define _MEASUREMENTS_INCLUDED
#include "localization_global_map.h"


using namespace std;

struct Particle{
    float xPos;
    float yPos;
    float thetaPos;
    float weight;
    Particle (): xPos(0), yPos(0), thetaPos(0), weight(1) {}

};

void getParticlesWeight(vector<Particle> &particles, LocalizationGlobalMap map, vector<pair<float, float>> laser_data, float max_distance);

void calculateIntrinsicParameters(LocalizationGlobalMap map, vector<pair<float, float>>, float max_distance, float pos_x, float pos_y, float lidar_orientation, float &z_hit, float &z_short, float &z_max, float &z_random, float &sigma_hit, float &lambda_short);
  
pair<float, float> particleToLidarConversion(float x_particle, float y_particle, float theta_particle, float lidar_x, float lidar_y);  

vector<pair<float, float>> calculateRealRange(LocalizationGlobalMap map, float translated_particle_x, float translated_particle_y, vector<pair<float, float>> laser_data, float lidar_orientation);

#endif
