#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

using namespace std;

struct Particle{
    float xPos;
    float yPos;
    float thetaPos;
    float weight;
    Particle (): xPos(0), yPos(0), thetaPos(0), weight(1) {}

};

void particlesWeight(vector<Particle> &particles, vector<float> &laser_ranges, float max_distance);

