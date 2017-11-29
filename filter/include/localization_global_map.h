
#include <string>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>

#ifndef _MAP_INCLUDED
#define _MAP_INCLUDED

using namespace std;

class LocalizationGlobalMap {
public:
    LocalizationGlobalMap();
    LocalizationGlobalMap(string _filname_map, float _cellSize);

    vector<vector<unsigned char> > global_map;
    vector<vector<double> > walls;
    double xMin;
    double yMin;
    double xMax;
    double yMax;

    pair<int, int> getCell(double x, double y);
    pair<float, float> getDistance(int x, int y);
    float getLineIntersection(float x1, float y1, float angle);


    pair<double,double> mapOffset;
    pair<double,double> mapScale;
    pair<size_t, size_t> gridSize;
    float cellSize;

    nav_msgs::OccupancyGrid visualGrid;

private:
    void createMap(string _filename_map);
    void createOccupancyGrid();
};
#endif
