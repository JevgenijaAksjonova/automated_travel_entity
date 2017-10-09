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
    GlobalPathPlanner(const string& mapFile, float p_cellSize, float p_robotRad);
    void updateMap();
    vector<pair<double,double> > getPath(pair<double,double> startCoord, pair<double,double> goalCoord);

private:
    float robotRad;
    float cellSize;
    //smoothObstaclesRad;
    //cellValueResolution = 1;

    vector<vector<unsigned char> > map;
    pair<double,double> mapOffset;
    pair<double,double> mapScale;
    pair<size_t,size_t> gridSize;

    pair<int, int> getCell(double x, double y);
    void addRobotRadiusToObstacles(double r);
    void setMap(string mapFile);
    //getLocation(i,j);
    int distanceHeuristic(const Node &a, const Node &b);
    vector<pair<int,int> > getPathGrid(pair<int,int> startCoord, pair<int, int> goalCoord);

};

#endif // GLOBAL_PATH_PLANNER_H
