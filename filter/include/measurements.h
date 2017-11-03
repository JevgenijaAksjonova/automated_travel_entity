#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

#include <localization_global_map.h> 

using namespace std;

struct Particle{
    float xPos;
    float yPos;
    float thetaPos;
    float weight;
    Particle (): xPos(0), yPos(0), thetaPos(0), weight(1) {}

};

void getParticlesWeight(vector<Particle> &particles, LocalizationGlobalMap map, vector<pair<float, float>> laser_data, float max_distance);



