
#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

#include <localization_global_map.h>


using namespace std;

LocalizationGlobalMap::LocalizationGlobalMap(string _filename_map, float _cellSize) {
    cellSize = _cellSize;
    createMap(_filename_map);
    createOccupancyGrid();
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


void LocalizationGlobalMap::createMap(string filename) {


    ifstream file;
    file.open(filename.c_str());
    if (!file.is_open()){
        return;
    }

    double max_num = numeric_limits<double>::infinity();
    double min_num = -numeric_limits<double>::infinity();
    string line;
    vector<vector<double> > walls;
    double x_min = max_num, y_min = max_num;
    double x_max = min_num, y_max = min_num;

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
        x_min = min(x_min, wall[0]);
        x_min = min(x_min, wall[2]);
        x_max = max(x_max, wall[0]);
        x_max = max(x_max, wall[2]);
        y_min = min(y_min, wall[1]);
        y_min = min(y_min, wall[3]);
        y_max = max(y_max, wall[1]);
        y_max = max(y_max, wall[3]);
    }

    mapOffset = pair<double,double>(x_min, y_min);
    mapScale = pair<double,double>(x_max - x_min, y_max - y_min);
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
