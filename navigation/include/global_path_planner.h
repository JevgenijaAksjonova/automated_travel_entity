/*
 *  global_path_planner.h
 *
 *
 *  Created on: Oct 9, 2017
 *  Authors:   Jevgenija Aksjonova
 *            jevaks <at> kth.se
 */

#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H 1

#include <iostream>
#include <string>
#include <vector>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32MultiArray.h"


using namespace std;

struct Node {
  int x;
  int y;
  double val;

  Node(int p_x, int p_y, double p_val);
  Node();
};

bool operator<(const Node &a, const Node &b);

bool operator==(const Node &a, const Node &b);

class GlobalPathPlanner
{
public:
    GlobalPathPlanner() {};
    GlobalPathPlanner(const string& mapFile, float p_cellSize, float p_robotRad);
    void updateMap();
    vector<pair<double,double> > getPath(pair<double,double> startCoord, pair<double,double> goalCoord);

    vector<vector<unsigned char> > map;
    pair<double,double> mapOffset;
    pair<double,double> mapScale;
    pair<size_t,size_t> gridSize;
    float cellSize;
    bool mapChanged;

    pair<int, int> getCell(double x, double y);
    int getDistance(pair<double,double> startCoord, pair<double,double> goalCoord);

    // exploration
    int explorationStatus; // 0 - initial; 1 - follow path; 2 - do not follow a path; 3 - finished
    vector<Node> nodes;
    vector<pair<double, double> > explorationPath;
    vector<pair<int, int> > nodeMarks;
    void explorationCallback(bool start_exploration, double x, double y);
    void explorationUpdate(double x, double y, double theta, int pathSize);

    // wall adding
    void updateMap(vector<double> wall);
    void newWallCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    void addRobotRadiusToPoint(pair<int, int> xy);

    //recovery
    void writeNodesToFile();
    void readNodesFromFile();
    void recovery();
    ros::Publisher statusPub;

private:
    float robotRad;
    //smoothObstaclesRad;
    //cellValueResolution = 1;

    void addRobotRadiusToObstacles(double r);
    void setMap(string mapFile);
    //getLocation(i,j);
    double distanceHeuristic(const Node &a, const Node &b);
    vector<pair<int,int> > getPathGrid(pair<int,int> startCoord, pair<int, int> goalCoord);
    double findClosestFreeCell(Node& goal,int maxD);
    void sampleNodesToExplore();
    void computeExplorationPath();
    void getExplorationPath(double x, double y);
    void recalculateExplorationPath(double x, double y);
};

#endif // GLOBAL_PATH_PLANNER_H
