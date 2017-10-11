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

#include <global_path_planner.h>

using namespace std;

Node::Node(int p_x, int p_y, double p_val) : x(p_x), y(p_y), val(p_val) {};
Node::Node() : x(-1), y(-1), val(-1) {};

bool operator<(const Node &a, const Node &b){
     return a.val > b.val;
};

bool operator==(const Node &a, const Node &b){
     return a.val == b.val;
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

    int w = ceil(r/cellSize);
    vector<vector<int> > f(2*w+1, vector<int>(2*w+1,0));
    for (int i = 0; i < 2*w+1; i++){
        for (int j = 0; j < 2*w+1; j++) {
            double x = (w-i)*cellSize;
            double y = (w-j)*cellSize;
            if (pow(x,2)+pow(y,2) <= pow(r,2)) {
                f[i][j] = 1;
            }
            //cout << f[i][j] << " ";
        }
        //cout << endl;
    }
    /*vector<vector<int> > sumMap(gridSize.first, vector<int>(gridSize.second, 0));
    for (size_t i = 0; i < gridSize.first; i++){
        for (size_t j = 0; j < gridSize.second; j++) {
            if (i == 0 && j == 0) {
               sumMap[i][j] = map[i][j];
            } else if (i > 0 && j == 0) {
               sumMap[i][j] = map[i][j] + sumMap[i-1][j];
            } else if (i == 0 && j > 0) {
               sumMap[i][j] = map[i][j] + sumMap[i][j-1];
            } else {
               sumMap[i][j] = map[i][j] + sumMap[i-1][j] + sumMap[i][j-1] - sumMap[i-1][j-1];
            }
        }
    }
    for (size_t i = 0; i < gridSize.first; i++){
        for (size_t j = 0; j < gridSize.second; j++) {
            int sumWindow = 0;
            if (i < w+1 && j < w+1) {
               sumWindow = sumMap[i+w][j+w];
            } else if (i >= w+1 && j < w+1) {
               sumWindow = sumMap[min(i+w,gridSize.first-1)][j+w]
                         - sumMap[i-w-1][j+w];
            } else if (i < w+1 && j >= w+1) {
               sumWindow = sumMap[i+w][min(j+w,gridSize.second-1)]
                         - sumMap[i+w][j-w-1];
            } else {
               sumWindow = sumMap[min(i+w,gridSize.first-1)][min(j+w,gridSize.second-1)]
                         - sumMap[i-w-1][min(j+w,gridSize.second-1)]
                         - sumMap[min(i+w,gridSize.first-1)][j-w-1]
                         + sumMap[i-w-1][j-w-1];
            }
            if (sumWindow > 0) {
                map[i][j] = 1;
            }
            //cout << (int)map[i][j] << " ";
        }
        //cout << endl;
    }*/
    vector<vector<int> > sumWindow(gridSize.first, vector<int>(gridSize.second, 0));
    for (int i = 0; i < gridSize.first; i++){
        for (int j = 0; j < gridSize.second; j++) {
            for (int di = -w; di < w+1; di++) {
                for (int dj = -w; dj < w+1; dj++){
                    if (i+di >= 0 && i+di < gridSize.first && j+dj >= 0 && j+dj < gridSize.second) {
                        sumWindow[i][j] += f[w+di][w+dj]*(int)map[i+di][j+dj];
                    }
                }
            }
            //cout << sumWindow[i][j] << " ";
        }
        //cout << endl;
    }
    for (int i = 0; i < gridSize.first; i++){
        for (int j = 0; j < gridSize.second; j++) {
            if (sumWindow[i][j] > 0) {
                map[i][j] = 1;
            }
            //cout << (int)map[i][j] << " ";
        }
        //cout << endl;
    }
}

void GlobalPathPlanner::setMap(string mapFile){

    ifstream mapFS; mapFS.open(mapFile.c_str());
    if (!mapFS.is_open()){
        return;
    }

    double max_num = numeric_limits<double>::infinity();
    double min_num = - numeric_limits<double>::infinity();
    string line;
    vector<vector<double> > walls;
    double x_min = max_num, y_min = max_num;
    double x_max = min_num, y_max = min_num;
    while (getline(mapFS, line)){
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
    //cout << "Grid Size = " <<  gridSize.first << " " << gridSize.second << endl;

    // fill the map
    map = vector<vector<unsigned char> >(gridSize.first, vector<unsigned char>(gridSize.second,0));
    double radius = max(robotRad, cellSize);
    for (size_t i = 0; i < walls.size(); i++) {
        double x1 = walls[i][0];
        double y1 = walls[i][1];
        double x2 = walls[i][2];
        double y2 = walls[i][3];
        double dx = x2 - x1;
        double dy = y2 - y1;
        size_t count = 0;
        while (pow(dx,2) + pow(dy,2) > pow(radius*2,2)/4.0) {
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

vector<pair<double,double> > GlobalPathPlanner::getPath(pair<double,double> startCoord, pair<double,double> goalCoord) {
    pair<int, int> startGrid = getCell(startCoord.first, startCoord.second);
    pair<int, int> goalGrid = getCell(goalCoord.first, goalCoord.second);
    vector<pair<int,int> > pathGrid = getPathGrid(startGrid, goalGrid);
    vector<pair<double, double> > path;
    for (size_t i = 0; i < pathGrid.size(); i++) {
        double x = mapOffset.first+(pathGrid[i].first+0.5)*cellSize;
        double y = mapOffset.second+(pathGrid[i].second+0.5)*cellSize;
        path.push_back(pair<double,double>(x,y));
    }
    return path;
}

/* A* algorithm */
vector<pair<int,int> > GlobalPathPlanner::getPathGrid(pair<int,int> startCoord, pair<int,int> goalCoord) {

    size_t nx = gridSize.first;
    size_t ny = gridSize.second;
    Node start = Node(startCoord.first, startCoord.second, 0);
    Node goal = Node(goalCoord.first, goalCoord.second, 0);

    if (map[goal.x][goal.y] > 0) {
       // not empty
       return vector<pair<int,int> >();
    }

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
        return vector<pair<int,int> >();
    }

    // trace path
    vector<pair<int,int> > path;
    Node node = goal;
    path.push_back(pair<int,int>(node.x,node.y));
    while (!(node.x == start.x && node.y == start.y)) {
        node = prev_node[node.x][node.y];
        path.push_back(pair<int,int>(node.x,node.y));
    }
    reverse(path.begin(),path.end());
    return path;
}

/*
int main() {
    string filename = "/home/ras/catkin_ws/src/ras_maze/ras_maze_map/maps/lab_maze_2017.txt";
    GlobalPathPlanner gpp(filename, 0.04, 0.15);
    //cout << "Initialization - done" << endl;
    //vector<pair<int,int> > res = gpp.getPath(pair<int,int>(5,7), pair<int,int>(61-7,63-7));
    //for (size_t i = 0; i < res.size(); i++) {
    //    cout << res[i].first << " " << res[i].second << endl;
    //}
    return 0;
}*/

