#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

#include <localization_global_map.h> 

using namespace std;

void getParticlesWeight(vector<Particle> &particles, LocalizationGlobalMap map, vector<pair<float, float>> laser_data, float max_distance);
