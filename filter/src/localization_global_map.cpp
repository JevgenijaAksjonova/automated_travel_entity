#define _USE_MATH_DEFINES
#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

#include <localization_global_map.h>


using namespace std;

LocalizationGlobalMap::LocalizationGlobalMap(){
}

LocalizationGlobalMap::LocalizationGlobalMap(string _filename_map, float _cellSize) {
    ROS_INFO("Inside map");
    cellSize = _cellSize;
    createMap(_filename_map);
    ROS_INFO("Before OccupancyGrid");

    //createOccupancyGrid();
    ROS_INFO("After OccupancyGrid");

}


pair<int, int> LocalizationGlobalMap::getCell(double x, double y) {
    int i = trunc((x - mapOffset.first)/cellSize);
    int j = trunc((y - mapOffset.second)/cellSize);
    return pair<int,int>(i,j);
}


pair<float, float> LocalizationGlobalMap::getDistance(int x, int y) {
    float i = float(x) * cellSize + mapOffset.first;
    float j = float(y) * cellSize + mapOffset.second;
    return pair<float, float>(i,j);
}

float LocalizationGlobalMap::getLineIntersection(float x1, float y1, float theta){

    float distance = 4;
    float x2 = (cos(theta)*distance) + x1;
    float y2 = (sin(theta)*distance) + y1;

    //ROS_INFO("----------------------x1 [%f] y1 [%f] x2 [%f] y2 [%f] theta [%f]", x1,y1,x2,y2,theta);

    float x3, y3, x4, y4;
    float d = 0;
    for(int i = 0; i < walls.size(); i++){
        //ROS_INFO("for i = [%d]",i);
        x3 = walls[i][0];
        y3 = walls[i][1];
        x4 = walls[i][2];
        y4 = walls[i][3];

        //ROS_INFO("Wall: x3 [%f] y3 [%f] x4 [%f] y4 [%f]", x3,y3,x4,y4);
        /*
        //Start of line check algorithm
        d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        // If d is zero, there is no intersection
        
        if (d == 0) continue;

        // Get the x and y
        float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
        float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
        float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

        ROS_INFO("Calculated x [%f] y [%f]",x,y);
        */
        float a1, a2, b1, b2;
        float x, y;
        if (x1 == x2 && x4 == x3){
            //ROS_INFO("cont 1");
            continue;
        } else if (x1 == x2) {
            x = x1;
            a2 = (y4-y3)/(x4-x3);
            b2 = (y3+y4)/2 - a2*(x3+x4)/2;
            y = a2*x + b2;
        } else if (x4 == x3) {
            x = x3;
            a1 = (y2-y1)/(x2-x1);
            b1 = (y1+y2)/2 - a1*(x1+x2)/2;
            y = a1*x+b1;
        } else {
            a1 = (y2-y1)/(x2-x1);
            a2 = (y4-y3)/(x4-x3);
            b1 = (y1+y2)/2 - a1*(x1+x2)/2;
            b2 = (y3+y4)/2 - a2*(x3+x4)/2;
            if(a2 == a1){
                //ROS_INFO("cont 2");
                continue;
            }
            x = (b2-b1)/(a1-a2);
            y = (a1*x) + b1;
        }
        //ROS_INFO("X = [%f], Y = [%f]", x, y);

    
        // Check if the x and y coordinates are within both lines
        float epsilon = 0.02;
        if ( x+epsilon < min(x1,x2) || x-epsilon > max(x1,x2)){
            //ROS_INFO("break1");
            continue;
        }

        if ( x+epsilon < min(x3,x4) || x-epsilon > max(x3,x4)){
            //ROS_INFO("break2");
            continue;
        }

        if ( y+epsilon < min(y1,y2) || y-epsilon > max(y1,y2)){
            //ROS_INFO("break3");
            continue;
        }

        if ( y+epsilon < min(y3,y4) || y-epsilon > max(y3,y4)){
            //ROS_INFO("break4");
            continue;
        }


        float temp_dist = sqrt((pow((x-x1), 2) + pow((y-y1),2))) - 0.009;
        //ROS_INFO("temp_dist = [%f]",temp_dist);
        if(temp_dist < distance){
            distance = temp_dist;
        }
    }
    // if(distance >3){
    //     ROS_INFO("Got 4!!");
    //     exit (EXIT_FAILURE);
    // }
    return distance;

}


void LocalizationGlobalMap::createMap(string filename) {


    ifstream file;
    file.open(filename.c_str());
    if (!file.is_open()){
        return;
    }

    double max_num = numeric_limits<double>::infinity();
    double min_num = -numeric_limits<double>::infinity();
    string line;
    xMin = max_num, yMin = max_num;
    xMax = min_num, yMax = min_num;

    while (getline(file, line)){
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
        xMin = min(xMin, wall[0]);
        xMin = min(xMin, wall[2]);
        xMax = max(xMax, wall[0]);
        xMax = max(xMax, wall[2]);
        yMin = min(yMin, wall[1]);
        yMin = min(yMin, wall[3]);
        yMax = max(yMax, wall[1]);
        yMax = max(yMax, wall[3]);
    }

    mapOffset = pair<double,double>(xMin, yMin);
    mapScale = pair<double,double>(xMax - xMin, yMax - yMin);
    gridSize = pair<size_t,size_t>(ceil(mapScale.first/cellSize), ceil(mapScale.second/cellSize));
    cout << "Grid Size = " <<  gridSize.first << " " << gridSize.second << endl;

    // fill the map
    global_map = vector<vector<unsigned char> >(gridSize.first, vector<unsigned char>(gridSize.second,0));
    double radius = 0.01;

    for (size_t i = 0; i < walls.size(); i++) {

        pair<int, int> firstPoint = getCell(walls[i][0], walls[i][1]);
        int x0 = firstPoint.first;
        int y0 = firstPoint.second;

        pair<int, int> secondPoint = getCell(walls[i][2], walls[i][3]);
        int x1 = secondPoint.first;
        int y1 = secondPoint.second;

        double dx = abs(x1 - x0);
        double dy = abs(y1 - y0);

        double err = 0;

        int x = x0;
        int y = y0;

        int sx = 1;
        int sy = 1;

        if (x0 > x1) {
            sx = -1;
        }

        if (y0 > y1) {
            sy = -1;
        }

        if(dx == 0) {
            while (y != y1) {
                global_map[x][y] = 1;
                y += sy;
            }
        } else if(dy == 0) {
            while (x != x1) {
                global_map[x][y] = 1;
                x += sx;
            }
        } else if(dx > dy) {
            err = dx / 2.0;
            while (x != x1) {
                global_map[x][y] = 1;
                err -= dy;
                if(err < 0) {
                    y += sy;
                    err += dx;
                }
                x += sx;
            }
        } else {
            err = dy / 2.0;
            while (y != y1) {
                global_map[x][y] = 1;
                err -= dx;
                if(err < 0) {
                    x += sx;
                    err += dy;
                }
                y += sy;
            }
        }
        global_map[x][y] = 1;
    }
}


void LocalizationGlobalMap::createOccupancyGrid() {
    visualGrid.header.frame_id = "/localization_map";
    visualGrid.header.stamp = ros::Time::now();

    visualGrid.info.map_load_time = ros::Time::now();
    visualGrid.info.resolution = cellSize;
    visualGrid.info.width = gridSize.first;
    visualGrid.info.height = gridSize.second;
    visualGrid.info.origin.position.x = 0.0;
    visualGrid.info.origin.position.y = 0.0;
    visualGrid.info.origin.position.z = 0.0;
    visualGrid.info.origin.orientation.x = 0.0;
    visualGrid.info.origin.orientation.y = 0.0;
    visualGrid.info.origin.orientation.z = 0.0;
    visualGrid.info.origin.orientation.w = 0.0;


    size_t nx = gridSize.first;
    size_t ny = gridSize.second;

    for (size_t j = 0; j < ny; j++) {
        for (size_t i = 0; i < nx; i++) {
            if (global_map[i][j] == 0) {
                visualGrid.data.push_back(0);
            } else {
                visualGrid.data.push_back(255);
            }
        }
    }
}

/*
int main(int argc, char** argv)
{

    ros::init(argc, argv, "world_map_node");

    std::string _filename_map = "/home/ras/catkin_ws/src/automated_travel_entity/world_map/maps/test.txt";
    //std::string _filename_map;

    ros::NodeHandle n("~");
    ros::Rate r(10);

    localization_grid_pub = n.advertise<nav_msgs::OccupancyGrid>("/localization_grid", 1);

    //ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

    readMapFile(_filename_map);
    addGrid();

    while(ros::ok()) {
        localization_grid_pub.publish(grid);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
*/
