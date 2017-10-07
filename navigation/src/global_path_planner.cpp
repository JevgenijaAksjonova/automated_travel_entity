/*
 *  global_path_planner.cpp
 *
 *
 *  Created on: Oct 6, 2017
 *  Authors:   Jevgenija Aksjonova
 *            jevaks <at> kth.se
 */

#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <limits>
#include <math.h>

using namespace std;

struct Node{
  int x;
  int y;
  double val;

  Node(int p_x, int p_y, double p_val) : x(p_x), y(p_y), val(p_val) {};
  Node() : x(-1), y(-1), val(-1) {};
};

bool operator<(const Node &a, const Node &b){
     return a.val > b.val;
};

bool operator==(const Node &a, const Node &b){
     return a.val == b.val;
};


class GlobalPathPlanner
{
public:
    GlobalPathPlanner(const string& mapFile, float p_cellSize, float p_robotRad);
    void updateMap();
    vector<pair<int,int>> getPath(pair<int,int> startCoord, pair<int, int> goalCoord);

private:
    float robotRad;
    float cellSize;
    //smoothObstaclesRad;
    //cellValueResolution = 1;

    vector<vector<unsigned char> > map;
    pair<double,double> mapOffset;
    pair<double,double> mapScale;
    pair<szie_t,size_t> gridSize;

    pair<int, int> getCell(double x, double y);
    void addRobotRadiusToObstacles(double r);
    void setMap(string mapFile);
    //getLocation(i,j);
    int distanceHeuristic(const Node &a, const Node &b);

};

GlobalPathPlanner::GlobalPathPlanner(const string& mapFile, float p_cellSize, float p_robotRad){
    cellSize = p_cellSize;
    robotRad = p_robotRad;
    setMap(mapFile);
}

pair<int, int> GlobalPathPlanner::getCell(double x, double y){
    int i = trunc((x - mapOffset.first)/cellSize);
    int j = trunc((y - mapOffset.second)/cellSize);
    return pair<int,int>(i,j);
}

void GlobalPathPlanner::addRobotRadiusToObstacles(double r){
    int w = 2*ceil(r/cellSize);
    vector<vector<unsigned char>> f(w, vector<int>(w,0));
    for (size_t i = 0; i < w; i++){
        for (size_t j = 0; j < w; j++) {
            double x = abs(i+0.5-w/2)*cellSize;
            double y = abs(j+0.5-w/2)*cellSize;
            if (pow(x,2)+pow(y,2) < r) {
                f[i][j] = 1;
            }
        }
    }
    vector<vector<int>> sumMap(gridSize.first, vector<int>(gridSize.second, 0));
    for (size_t i = 0; i < gridSize.first; i++){
        for (size_t j = 0; j < gridSize.second; j++) {
            if (i == 0 && j = 0) {
               sumMap[i][j] = map[i][j];
            } else if (i > 0 && j = 0) {
               sumMap[i][j] = map[i][j] + sumMap[i-1][j];
            } else if (i = 0 && j > 0) {
               sumMap[i][j] = map[i][j] + sumMap[i][j-1];
            } else {
               sumMap[i][j] = map[i][j] + sumMap[i-1][j] + sumMap[i][j-1] - sumMap[i-1][j-1];
            }
        }
    }
    for (size_t i = 0; i < gridSize.first; i++){
        for (size_t j = 0; j < gridSize.second; j++) {

        }
    }
}

void GlobalPathPlanner::setMap(string mapFile){

    double max_num = numeric_limits<double>::infinity();
    double min_num = - numeric_limits<double>::infinity();
    string line;
    vector<vector<double>> walls;
    double x_min = min_num, y_min = min_num;
    double x_max = max_num, y_max = max_num;
    while (getline(mapFile, line)){
        if (line[0] == '#') {
            // comment -> skip
            continue;
        }
        vector<double> wall(4, max_num);

        istringstream line_stream(line);
        // x1, y1, x2, y2
        line_stream >> wall[0] >> wall[1] >> wall[2] >> wall[3];

        if ((wall[0] == max_num) || (wall[1] == max_num) || (wall[2] == max_num) || (wall[3] == max_num)){
            continue;
        }
        walls.push_back(wall);
        x_min = min(x_min,wall[0]);
        x_min = min(x_min,wall[2]);
        x_max = max(x_max,wall[0]);
        x_max = max(x_max,wall[2]);
        y_min = min(y_min,wall[1]);
        y_min = min(y_min,wall[3]);
        y_max = max(y_max,wall[1]);
        y_max = max(y_max,wall[3]);
    }

    mapOffset = pair<double,double>(x_min,y_min);
    mapScale = pair<double,double>(x_max-x_min,y_max-y_min);
    gridSize = pair<size_t,size_t>(ceil(mapScale.first/cellSize), ceil(mapScale.second/cellSize));

    // fill the map
    map = vector<vector<unsigned char>>(gridSize.first, vector<unsigned char>(gridSize.second,0));
    double radius = max(robotRad, cellSize);
    for (size_t i = 0; i < walls.size(); i++) {
        double x1 = walls[i][0];
        double y1 = walls[i][1];
        double x2 = walls[i][2];
        double y2 = walls[i][3];
        double dx = x2 - x1;
        double dy = y2 - y1;
        size_t count = 0;
        while (pow(dx,2) + pow(dy,2) > pow(radius*2,2) {
            dx /= 2;
            dy /= 2;
            count++;
        }
        for (size_t c = 0; c < pow(2,count)+1; c++) {
            pair<int, int> cell = getCell(x1 + c*dx, y1 + c*dy);
            map[cell.first][cell.second] = 1;
        }
    }
    addRobotRadiusToObstacles(radius);
}

void GlobalPathPlanner::updateMap(){

}

int GlobalPathPlanner::distanceHeuristic(const Node &a, const Node &b){
    // Manhattan distance
    return abs(b.x - a.x) + abs(b.y - a.y);
}

/* A* algorithm */
vector<pair<int,int>> GlobalPathPlanner::getPath(pair<int,int> startCoord, pair<int,int> goalCoord) {

    size_t nx = gridSize.first;
    size_t ny = gridSize.second;
    Node start = Node(startCoord.first, startCoord.second, 0);
    Node goal = Node(goalCoord.first, goalCoord.second, 0);
    start.val = distanceHeuristic(start, goal);
    priority_queue<Node> nodes;
    vector<vector<Node> > prev_node(nx, vector<Node>(ny,Node()));
    prev_node[start.x][start.y] = start;
    nodes.push(start);
    size_t step = 0;
    while (!nodes.empty()){
        Node position = nodes.top();
        nodes.pop();
        if (position == goal) {
            break;
        }
        int dh = distanceHeuristic(position, goal);
        for (int dx = -1; dx <= 1; dx += 1) {
            for (int dy = -1; dy <= 1; dy += 1) {
                if (abs(dx) + abs(dy) == 1 &&
                    position.x + dx >= 0 && position.y + dy >= 0 &&
                    position.x + dx < nx && position.y + dy < ny &&
                    map[position.x + dx][position.y + dy] == 0 &&
                    prev_node[position.x + dx][position.y + dy].x == -1) {
                    Node new_node(position.x + dx, position.y + dy, 0);
                    new_node.val = position.val - dh + 1 + distanceHeuristic(new_node, goal);
                    nodes.push(new_node);
                    prev_node[new_node.x][new_node.y] = position;
                }
            }
        }
        step++;
    }

    // path not found
    if (prev_node[goal.x][goal.y].x == -1) {
        return vector<pair<int,int>>();
    }

    // trace path
    vector<pair<int,int>> path;
    Node node = goal;
    path.push_back(pair<int,int>(node.x,node.y));
    while (!(node.x == start.x && node.y == start.y)) {
        node = prev_node[node.x][node.y];
        path.push_back(pair<int,int>(node.x,node.y));
    }
    reverse(path.begin(),path.end());
    return path;
}

int main() {
    string filename = "/home/ras/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
    GlobalPathPlanner gpp(filename, 0.1, 0.0);
    vector<pair<int,int>> res = gpp.getPath(pair<int,int>(0,0), pair<int,int>(6,5));
    for (size_t i = 0; i < res.size(); i++) {
        cout << res[i].first << " " << res[i].second << endl;
    }

    return 0;
}
