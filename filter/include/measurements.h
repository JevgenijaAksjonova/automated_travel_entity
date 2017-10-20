#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

using namespace std;

pair<float, float> localToWorldCoordinates(float x_particle, float y_particle, float theta_particle, float lidar_x, float lidar_y, float lidar_orientation, float range_laser, float angle_laser);
pair<int, int> getClosestWallCoordinates(vector<vector<unsigned char> > global_map, float cellSize, float angle, int x_base, int y_base);

float distancesToRange(pair<float, float> wall_distances, pair<float, float> base_distances);
