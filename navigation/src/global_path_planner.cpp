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
#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

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
    explorationStatus = 0;
    mapChanged = false;
    //getExplorationPath();

}

pair<int, int> GlobalPathPlanner::getCell(double x, double y){
    int i = trunc((x - mapOffset.first)/cellSize);
    int j = trunc((y - mapOffset.second)/cellSize);
    return pair<int,int>(i,j);
}

int GlobalPathPlanner::getDistance(pair<double,double> startCoord, pair<double,double> goalCoord) {
    vector<pair<double,double> > path = getPath(startCoord, goalCoord);
    return path.size();
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

void GlobalPathPlanner::newWallCallback(const std_msgs::Float32MultiArray::ConstPtr& array){
    vector<double> wall;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
        wall.push_back(*it);
    }
    if(wall.size() != 4){
        ROS_INFO("WALL HAS WERID DIMENSIONS %lu", wall.size());
    }
    ROS_INFO("RECIVED NEW WALL --- [%f] [%f] [%f] [%f] ", wall[0], wall[1], wall[2], wall[3]);
    updateMap(wall);
}

void GlobalPathPlanner::updateMap(vector<double> wall){

    double x1 = wall[0];
    double y1 = wall[1];
    double x2 = wall[2];
    double y2 = wall[3];
    double dx = x2 - x1;
    double dy = y2 - y1;
    size_t count = 0;
    while (pow(dx,2) + pow(dy,2) > pow(robotRad*2,2)/4.0) {
        dx /= 2;
        dy /= 2;
        count++;
    }
    for (size_t c = 0; c < pow(2,count)+1; c++) {
        pair<int, int> cell = getCell(x1 + c*dx, y1 + c*dy);
        addRobotRadiusToPoint(cell);
    }
    mapChanged = true;


}

void GlobalPathPlanner::addRobotRadiusToPoint(pair<int, int> xy){

    int radCell = robotRad/cellSize;
    int maxX = static_cast<int>(gridSize.first);
    int maxY = static_cast<int>(gridSize.second);

    int startX = max(xy.first - radCell, 0);
    int startY = max(xy.second - radCell, 0);
    int endX = min(xy.first + radCell, maxX-1);
    int endY = min(xy.second + radCell, maxY-1);
    float distance;

    for (int i = startX; i <=endX; i++){
        for (int j = startY; j <=endY; j++){
            if(pow((i-xy.first)*cellSize,2) + pow((j-xy.second)*cellSize,2) <= pow(robotRad,2)){
                map[i][j] = 1;
            }
        }
    }



    

}

double GlobalPathPlanner::distanceHeuristic(const Node &a, const Node &b){
    // Manhattan distance
    //return abs(b.x - a.x) + abs(b.y - a.y);
    // Euclidean distance
    return pow(pow(b.x - a.x,2) + pow(b.y - a.y,2),0.5);
}


// return distance to the closest free cell, changes goal to that cell
double GlobalPathPlanner::findClosestFreeCell(Node& goal,int maxD){
    priority_queue<Node> cells;
    //cout << "Cell values " << endl;
    for (int dx = -maxD; dx < maxD+1; dx++) {
        for (int dy = -maxD; dy < maxD+1; dy++) {
            if (goal.x+dx >= 0 && goal.x+dx < gridSize.first &&
                goal.y+dy >= 0 && goal.y+dy < gridSize.second &&
                map[goal.x+dx][goal.y+dy] == 0) {
                Node cell(goal.x+dx, goal.y+dy,0);
                cell.val = distanceHeuristic(goal, cell);
                //cout << cell.val <<  " ";
                cells.push(cell);
            }
        }
    }
    if (!cells.empty()) {
        Node top = cells.top();
        goal = top;
        //cout << "There are free cells! Top " << top.x << " " << top.y << " "<< top.val<<endl;
        return top.val;
    } else {
        return maxD+1;
    }
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
// will return empty vector if path not found, and vector of length 1 if start == goal
vector<pair<int,int> > GlobalPathPlanner::getPathGrid(pair<int,int> startCoord, pair<int,int> goalCoord) {

    size_t nx = gridSize.first;
    size_t ny = gridSize.second;
    Node start = Node(startCoord.first, startCoord.second, 0);
    Node goal = Node(goalCoord.first, goalCoord.second, 0);

    // handle situations, when startCoord or goalCoord are in non-empty positions
    int maxD = ceil(robotRad/cellSize);
    if (map[start.x][start.y] == 1) {
       // cell is not empty, find the closest, which is within robotRad
       double dist = findClosestFreeCell(start, maxD);
       if (dist*cellSize > robotRad) {
           return vector<pair<int,int> >();
       } else {
           stringstream s;
           s << "Search path FROM the closest point within the distance = " << dist << endl;
           ROS_INFO("%s/n", s.str().c_str());
       }
    }

    //cout <<"GPP started, goal cell: "<< goal.x <<  " " <<goal.y << endl;
    double distanceTol = 0;
    if (map[goal.x][goal.y] == 1) {
       //cout <<"Cell is not empty! " << robotRad <<endl;
       // cell is not empty, find the closest, which is within robotRad
       Node newGoal = goal;
       distanceTol = findClosestFreeCell(newGoal, maxD);
       if (distanceTol*cellSize > robotRad) {
           return vector<pair<int,int> >();
       } else {
           stringstream s;
           s << "Search path TO the closest point within the distance = " << distanceTol << endl;
           ROS_INFO("%s/n", s.str().c_str());
       }
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
        int dh = distanceHeuristic(position, goal);
        //cout << position.x << " " << position.y << " " << dh << " " << endl;
        if (dh <= distanceTol) {
            goal = position;
            break;
        }
        for (int dx = -1; dx <= 1; dx += 1) {
            for (int dy = -1; dy <= 1; dy += 1) {
                if (abs(dx) + abs(dy) == 1 &&
                    position.x + dx >= 0 && position.y + dy >= 0 &&
                    position.x + dx < nx && position.y + dy < ny &&
                    map[position.x + dx][position.y + dy] == 0 &&
                    prev_node[position.x + dx][position.y + dy].x == -1)
                {
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

void GlobalPathPlanner::sampleNodesToExplore() {

    double cellSizeL = 0.5;
    pair<int, int> gridSizeL(ceil(mapScale.first/cellSizeL), ceil(mapScale.second/cellSizeL));
    for (int i = 0; i < gridSizeL.first; i++) {
        for(int j = 0; j < gridSizeL.second; j++) {
            pair<int, int> coord = getCell((i+0.5)*cellSizeL,(j+0.5)*cellSizeL);
            Node cell(coord.first,coord.second,0);
            int maxD = round(cellSizeL/cellSize);
            double distance = findClosestFreeCell(cell,maxD);
            if (distance <= maxD) {
                nodes.push_back(cell);
            }
        }
    }
}

void GlobalPathPlanner::computeExplorationPath() {

    cout << "Compute exploration path " << endl;

    auto start = chrono::high_resolution_clock::now();
    // the first node should be a starting location
    // remove all other nodes, which cant be reached from it
    vector<int> edges1;
    edges1.push_back(0);
    int i = 1;
    while(i < nodes.size()) {
        vector<pair<int, int> > path = getPathGrid(pair<int,int>(nodes[0].x,nodes[0].y), pair<int,int>(nodes[i].x,nodes[i].y));
        if (path.size()>0) {
            edges1.push_back(path.size());
            i++;
        } else {
            nodes.erase(nodes.begin()+i);
        }
    }

    // find distance between all reachable nodes
    //vector<vector<int> > edges(nodes.size(),vector<int>(nodes.size(),-1));
    // fill in known values
    //edges[0] = edges1;
    //for (int i = 1; i < nodes.size(); i++) {
    //    edges[i][0] = edges[0][i];
    //}
    // compute others
    //for (int i = 1; i < nodes.size(); i++) {
    //    for (int j = i; j < nodes.size(); j++) {
    //        if (nodes[i].x == nodes[j].x && nodes[i].y == nodes[j].y ) {
    //            edges[i][j] = 1;
    //            edges[j][i] = 1;
    //        } else /*if (abs(nodes[i].x-nodes[j].x) < 2*cellSize && abs(nodes[i].y - nodes[j].y) < 2*cellSize) */{
    //            vector<pair<int, int> > path = getPathGrid(pair<int,int>(nodes[i].x,nodes[i].y), pair<int,int>(nodes[j].x,nodes[j].y));
    //            if (path.size() > 0) {
    //                edges[i][j] = path.size();
    //                edges[j][i] = path.size();
    //            }
    //        }
    //    }
    //}*/
    //auto end= chrono::high_resolution_clock::now();
    //chrono::duration<double> elapsed = end-start;
    //stringstream s;
    //s << "Time to compute the distance matrix = " << elapsed.count()<< endl;
    //ROS_INFO("%s/n", s.str().c_str());

    vector<int> visited(nodes.size(),0);
    vector<int> path;
    int count = 1;
    i = 0;
    path.push_back(i);
    visited[i] = 1;
    while (count < nodes.size()) {
        int jMin = -1;
        vector<int> edges(nodes.size(),-1);
        if (i ==0) {
            edges = edges1;
        }
        for (int j = 0; j < nodes.size(); j++) {
            if (visited[j] ==0) {
                vector<pair<int, int> > path = getPathGrid(pair<int,int>(nodes[i].x,nodes[i].y), pair<int,int>(nodes[j].x,nodes[j].y));
                if (path.size() > 0) {
                    edges[j] = path.size();
                    if (jMin == -1 || edges[j] < edges[jMin]) {
                        jMin = j;
                    }
                }
            }
        }
        count++;
        i = jMin;
        path.push_back(i);
        visited[i] = 1;
    }
    auto end= chrono::high_resolution_clock::now();
    elapsed = end-start;
    s.str("") ;
    s << "Time to find greedy path = " << elapsed.count()<< endl;
    ROS_INFO("%s/n", s.str().c_str());


    vector<pair<int,int> > pathGrid;
    cout << "Path : "<< endl;
    nodeMarks.push_back(pair<int,int>(path[0],0));
    for (int i = 0; i < path.size()-1; i++) {
        cout << path[i+1] << " ";
        vector<pair<int, int> > part = getPathGrid(pair<int,int>(nodes[path[i]].x,nodes[path[i]].y), pair<int,int>(nodes[path[i+1]].x,nodes[path[i+1]].y));
        pathGrid.insert(pathGrid.end(), part.begin(), part.end());
        nodeMarks.push_back(pair<int,int>(path[i+1], pathGrid.size()-1));
    }

    int pathSize = pathGrid.size();
    for (int i = 0; i < nodeMarks.size(); i++) {
        nodeMarks[i].second = pathSize - nodeMarks[i].second;
    }

    for (size_t i = 0; i < pathGrid.size(); i++) {
        double x = mapOffset.first+(pathGrid[i].first+0.5)*cellSize;
        double y = mapOffset.second+(pathGrid[i].second+0.5)*cellSize;
        explorationPath.push_back(pair<double,double>(x,y));
    }

}

// generates nodes and finds an exploration path through them
void GlobalPathPlanner::getExplorationPath(double x, double y) {

    string msg = "Generating an exploration path ...... ";
    ROS_INFO("%s/n", msg.c_str());

    pair<int, int> cell = getCell(x,y);
    Node startNode(cell.first,cell.second,0);

    sampleNodesToExplore();
    nodes.insert(nodes.begin(),startNode);

    computeExplorationPath();

}

void GlobalPathPlanner::recalculateExplorationPath(double x, double y) {
    if (!mapChanged) {
        pair<double, double>  pathStart = explorationPath[0];
        pair<double, double> location(x,y);
        cout << "Recalculate exploration, map did not change "<< x << " "<< y << " to "<< pathStart.first << " " << pathStart.second << endl;
        vector<pair<double, double> > path = getPath(location, pathStart);
        cout << "Path size " << path.size() << endl;
        explorationPath.insert(explorationPath.begin(),path.begin(), path.end());
        cout << "Exploration path size " << explorationPath.size() << endl;
    } else {
        cout << "Recalculate exploration, map changed" << endl;
        // delete nodes, which are already visited
        int pathSize = explorationPath.size();
        int i = 0;
        vector<int> nodesToErase;
        while (nodeMarks[i].second > pathSize) {
            nodesToErase.push_back(nodeMarks[i].first);
            i++;
        }
        // move and possibly delete nodes which were influenced by wall adding
        int maxD = round(robotRad/cellSize);
        for (int i = 0; i < nodes.size(); i++) {
            Node n = nodes[i];
            double distance = findClosestFreeCell(n,maxD);
            if (distance > maxD) {
                nodesToErase.push_back(i);
            } else {
                nodes[i] = n;
            }
        }

        cout << "Nodes to erase " << nodesToErase.size() << endl;
        if (nodesToErase.size()>0) {
            sort(nodesToErase.begin(), nodesToErase.end());
            for(int i = nodesToErase.size()-1; i>=0; i--) {
                nodes.erase(nodes.begin()+ nodesToErase[i]);
            }
        }
        explorationPath.clear();
        nodeMarks.clear();
        cout << "nodes left "<< nodes.size() << endl;

        pair<int, int> cell = getCell(x,y);
        Node startNode(cell.first,cell.second,0);
        nodes.insert(nodes.begin(),startNode);
        computeExplorationPath();
        mapChanged = false;
    }
}

void GlobalPathPlanner::explorationUpdate(double x, double y, double theta, int pathSize) {
    // erase part of the path, already explored
    int offset = explorationPath.size() - pathSize;
    explorationPath.erase(explorationPath.begin(),explorationPath.begin()+offset);
}

void GlobalPathPlanner::explorationCallback(bool start_exploration, double x, double y){
    if (start_exploration) {
        if (explorationStatus == 0) {
            getExplorationPath(x, y);
            explorationStatus = 1;
        } else {
            recalculateExplorationPath(x, y);
            explorationStatus = 1;
        }
    } else {
        // stop exploration
        explorationStatus = 2;
    }
}

//vectorexplorationPath(alreadyexplored path)


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

