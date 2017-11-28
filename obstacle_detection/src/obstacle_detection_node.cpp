#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <math.h>

#include <project_msgs/depth.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class ObstaclePublisher
{
public:
  float HEIGHT_LOWER_THRESHOLD;
  float HEIGHT_UPPER_THRESHOLD;
  float DISTANCE_THRESHOLD;
  float NBINS;

  float START_THETA;
  float END_THETA;

  float INCREMENT_SIZE;

  float BIN_THRESHOLD;

  bool PUBLISH_MARKERS;

  ros::NodeHandle n;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub;

  // Create a ROS publisher for the output point cloud
  ros::Publisher pub;

  // ROS publisher for obstacles found
  ros::Publisher obstacle_publisher;

  // ROS publisher for visualization in rviz
  ros::Publisher bin_publisher;

  // Sent vector for obstacle avoidance
  //std::vector<std::pair<float, float>> obstacles;
  sensor_msgs::PointCloud2 transformed_pc;
  sensor_msgs::PointCloud2 original_pc;

  project_msgs::depth obstacles_found;

  ObstaclePublisher()
  {
    n = ros::NodeHandle("~");

    sub = n.subscribe("/camera/depth/points", 1, &ObstaclePublisher::pointCloudCallback, this);

    pub = n.advertise<sensor_msgs::PointCloud2>("obstacle/output", 1);

    obstacle_publisher = n.advertise<project_msgs::depth>("/depth", 1);

    bin_publisher = n.advertise<visualization_msgs::MarkerArray>("/visual_bins", 1);

    HEIGHT_LOWER_THRESHOLD = 0.02;
    HEIGHT_UPPER_THRESHOLD = 0.04;
    DISTANCE_THRESHOLD = 0.4;
    NBINS = 180;

    START_THETA = M_PI / 2;
    END_THETA = -M_PI / 2;

    INCREMENT_SIZE = M_PI / NBINS;

    PUBLISH_MARKERS = true;

    BIN_THRESHOLD = 100;

  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    original_pc = *cloud_msg;

    // Transform cloud
    listener.lookupTransform("/base_link", "/camera_depth_optical_frame", ros::Time(0), transform);
    pcl_ros::transformPointCloud("/camera_depth_optical_frame", *cloud_msg, transformed_pc, listener);

    // Publish the data
    removeUnwantedData();

    if(PUBLISH_MARKERS) {
      visualizeBins();
    }

    pub.publish(transformed_pc);

    obstacle_publisher.publish(obstacles_found);
  }

  void removeUnwantedData()
  {
    pcl::PointCloud<pcl::PointXYZ> old_depth;
    pcl::PointCloud<pcl::PointXYZ> depth;
    pcl::fromROSMsg(original_pc, old_depth);

    pcl_ros::transformPointCloud(old_depth, depth, transform);

    std::vector<int> bins(NBINS, 0);
    std::vector<float> sum_dist(NBINS, 0);
    std::vector<float> avg_dist(NBINS, 0);

    std::vector<float> found_ranges;
    std::vector<float> found_angles;
    

    for (int i = depth.width - 1; i > 0; i--)
    {
      for (int j = 0; j < depth.height; j++)
      {
        pcl::PointXYZ depth_point = depth.at(i, j);

        //ROS_INFO("X: [%d], Y: [%d]", i, j);

        float x = depth_point._PointXYZ::data[0];
        float y = depth_point._PointXYZ::data[1];
        float z = depth_point._PointXYZ::data[2];

        if(!isnan(x) && !isnan(y) && !isnan(z)) {
          if (z > HEIGHT_LOWER_THRESHOLD && z < HEIGHT_UPPER_THRESHOLD)
          {
            double rad = Radius(x, y);
            if (rad < DISTANCE_THRESHOLD)
            {
              int bin = 0;
              double theta = Theta(x, y);

              bin = int(std::floor((theta + START_THETA) / INCREMENT_SIZE));

              bins[bin] += 1;
              sum_dist[bin] += rad;
            }
          }
        }
      }
    }

    for(int i = 0; i < sum_dist.size(); i++) {
      if(sum_dist[i] > 0 && bins[i] > BIN_THRESHOLD) {
        avg_dist[i] = sum_dist[i] / bins[i];

        found_ranges.push_back(avg_dist[i]);
        found_angles.push_back(i * INCREMENT_SIZE - START_THETA);
      }
    }
    distances = avg_dist;

    obstacles_found.ranges = found_ranges;
    obstacles_found.angles = found_angles;

    answer_bins = bins;
  }

  double Radius(double x, double y)
  {
    double rad;
    rad = sqrt((pow(x, 2)) + (pow(y, 2)));
    return rad;
  }

  double Theta(double x, double y)
  {
    double Thta;

    // TODO: USE ATAN2 IN THE FUTURE
    Thta = atan(y / x);
    return Thta;
  }

  void visualizeBins()
  {
    ros::Time current_time = ros::Time::now();

    visualization_msgs::MarkerArray all_bins;
    visualization_msgs::Marker bin;

    bin.header.stamp = current_time;
    bin.header.frame_id = "/base_link";

    bin.ns = "all_bins";
    bin.type = visualization_msgs::Marker::CUBE;
    bin.action = visualization_msgs::Marker::ADD;

    bin.pose.position.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    bin.color.r = 0.0f;
    bin.color.g = 0.0f;
    bin.color.b = 1.0f;

    float size = 0.01;

    int id = 0;
    for (int i = 0; i < answer_bins.size(); i++)
    {
      if(answer_bins[i] > BIN_THRESHOLD) {
        float theta = (i * INCREMENT_SIZE) - START_THETA;
        bin.pose.position.x = cos(theta) * distances[i];
        bin.pose.position.y = sin(theta) * distances[i];
        bin.color.a = 1.0;

        //bin.scale.z = 0.001 * answer_bins[i];
        //ROS_INFO("Bin: [%d]", i);
        //ROS_INFO("X[%f], Y[%f]", cos(theta) * 0.3, sin(theta) * 0.3);
      } else {
        bin.pose.position.x = 0;
        bin.pose.position.y = 0;
        bin.color.a = 0;
      }
      

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      bin.scale.y = size;
      bin.scale.x = size;
      bin.scale.z = size;

      bin.id = id;
      id++;

      all_bins.markers.push_back(bin);
    }

    bin_publisher.publish(all_bins);
  }

private:
  tf::TransformListener listener;
  tf::StampedTransform transform;

  std::vector<int> answer_bins;
  std::vector<float> distances;
};

int main(int argc, char **argv)
{

  ROS_INFO("Spinning!");

  // Initialize ROS
  ros::init(argc, argv, "obstacle_detection");

  ObstaclePublisher obs;
  ros::Rate loop_rate(10);

  int count = 0;
  while (obs.n.ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}