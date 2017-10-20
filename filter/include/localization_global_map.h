
#include <string>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>


using namespace std;

class LocalizationGlobalMap {
public:
    LocalizationGlobalMap();
    LocalizationGlobalMap(string _filname_map, float _cellSize);

    vector<vector<unsigned char> > global_map;
    pair<int, int> getCell(double x, double y);
    pair<float, float> getDistance(int x, int y);

    pair<double,double> mapOffset;
    pair<double,double> mapScale;
    pair<size_t, size_t> gridSize;
    float cellSize;

    nav_msgs::OccupancyGrid visualGrid;

private:
    void createMap(string _filename_map);
    void createOccupancyGrid();
};
