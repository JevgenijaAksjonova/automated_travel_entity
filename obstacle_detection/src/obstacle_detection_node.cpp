#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

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
  float CONNECTION_THRESHOLD;
  float WALL_LENGTH_THRESHOLD;
  float ANGULAR_VELOCITY_THRESHOLD;

  float NBINS;

  float START_THETA;
  float END_THETA;

  float INCREMENT_SIZE;

  float BIN_THRESHOLD;

  bool PUBLISH_MARKERS;

  ros::NodeHandle n;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub;

  ros::Subscriber filter_subscriber;



  // ROS publisher for visualization in rviz
  ros::Publisher bin_publisher;
  ros::Publisher obstacle_wall_pub;

  ros::Publisher batteries_publisher;

  tf::TransformListener listener_2;
  project_msgs::depth obstacles_found;

  std::vector<std::pair<std::pair<float, float>, std::pair<float, float> > > wall_segments;

  float angular_vel;

  tf::StampedTransform transform;

  pcl::PCLPointCloud2 cloud_filtered;

  // Sent vector for obstacle avoidance
  //std::vector<std::pair<float, float>> obstacles;
  sensor_msgs::PointCloud2 transformed_pc;
  sensor_msgs::PointCloud2 original_pc;

  bool point_cloud_received;

  ros::Time cloud_time;

  ObstaclePublisher()
  {
    n = ros::NodeHandle("~");

    sub = n.subscribe("/camera/depth/points", 1, &ObstaclePublisher::pointCloudCallback, this);

    filter_subscriber = n.subscribe("/filter", 1, &ObstaclePublisher::positionCallback, this);

    bin_publisher = n.advertise<visualization_msgs::MarkerArray>("/visual_bins", 1);
    obstacle_wall_pub = n.advertise<visualization_msgs::MarkerArray>("/visual_obstacle_walls", 1);

    batteries_publisher = n.advertise<std_msgs::Float32MultiArray>("/batteries_found", 1);

    HEIGHT_LOWER_THRESHOLD = 0.02;
    HEIGHT_UPPER_THRESHOLD = 0.04;
    DISTANCE_THRESHOLD = 0.4;
    CONNECTION_THRESHOLD = 0.04;
    WALL_LENGTH_THRESHOLD = 0.05;
    ANGULAR_VELOCITY_THRESHOLD = 0.7;

    NBINS = 180;

    PUBLISH_MARKERS = true;

    BIN_THRESHOLD = 100;

    START_THETA = M_PI / 2;
    END_THETA = -M_PI / 2;

    INCREMENT_SIZE = M_PI / NBINS;

    angular_vel = 0;

    point_cloud_received = false;

    if (!n.getParam("/obstacle_detection/thresholds/HEIGHT_LOWER_THRESHOLD", HEIGHT_LOWER_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 1");
      exit(EXIT_FAILURE);
    }
    if (!n.getParam("/obstacle_detection/thresholds/HEIGHT_UPPER_THRESHOLD", HEIGHT_UPPER_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 2");
      exit(EXIT_FAILURE);
    }
    if (!n.getParam("/obstacle_detection/thresholds/BIN_THRESHOLD", BIN_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 3");
      exit(EXIT_FAILURE);
    }
    if (!n.getParam("/obstacle_detection/thresholds/DISTANCE_THRESHOLD", DISTANCE_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 4");
      exit(EXIT_FAILURE);
    }
    if (!n.getParam("/obstacle_detection/thresholds/CONNECTION_THRESHOLD", CONNECTION_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 5");
      exit(EXIT_FAILURE);
    }
    if (!n.getParam("/obstacle_detection/thresholds/WALL_LENGTH_THRESHOLD", WALL_LENGTH_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 6");
      exit(EXIT_FAILURE);
    }
    if (!n.getParam("/obstacle_detection/thresholds/ANGULAR_VELOCITY_THRESHOLD", ANGULAR_VELOCITY_THRESHOLD))
    {
      ROS_ERROR("Obstacle detection failed to detect thresholds parameter 7");
      exit(EXIT_FAILURE);
    }

    if (!n.getParam("/obstacle_detection/visual/PUBLISH_MARKERS", PUBLISH_MARKERS))
    {
      ROS_ERROR("Obstacle detection failed to detect visual parameter 1");
      exit(EXIT_FAILURE);
    }
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
    // Container for original & filtered data
    original_pc = *cloud_msg;
    point_cloud_received = true;
    cloud_time = cloud_msg->header.stamp;
  }

  void positionCallback(const nav_msgs::Odometry::ConstPtr& msg) 
  {
    angular_vel = abs(msg->twist.twist.angular.z);
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

    for (int i = depth.width - 1; i > 0; i-= 5)
    {
      for (int j = 0; j < depth.height; j+=5)
      {
        pcl::PointXYZ depth_point = depth.at(i, j);

        float x = depth_point._PointXYZ::data[0];
        float y = depth_point._PointXYZ::data[1];
        float z = depth_point._PointXYZ::data[2];

        if (!isnan(x) && !isnan(y) && !isnan(z))
        {
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

    for (int i = 0; i < sum_dist.size(); i++)
    {
      if (sum_dist[i] > 0 && bins[i] > BIN_THRESHOLD)
      {
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

    bin.header.stamp = cloud_time;
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
      if (answer_bins[i] > BIN_THRESHOLD)
      {
        float theta = (i * INCREMENT_SIZE) - START_THETA;
        bin.pose.position.x = cos(theta) * distances[i];
        bin.pose.position.y = sin(theta) * distances[i];
        bin.color.a = 1.0;
      }
      else
      {
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

  void sendBatteries() {

    std_msgs::Float32MultiArray batteries;

    geometry_msgs::PointStamped ptStart_trans;
    geometry_msgs::PointStamped ptEnd_trans;

    geometry_msgs::PointStamped ptStart;
    geometry_msgs::PointStamped ptEnd;


    ptStart.header.frame_id = "base_link";
    ptEnd.header.frame_id = "base_link";


    for(int i = 0; i < wall_segments.size(); i++) {

      ptStart.point.x = wall_segments[i].first.first;
      ptStart.point.y = wall_segments[i].first.second;
      ptEnd.point.x = wall_segments[i].second.first;
      ptEnd.point.y = wall_segments[i].second.second;
      //ros::Time now = ros::Time::now();
      try {
        listener_2.waitForTransform("odom", "base_link", cloud_time, ros::Duration(0.1));

        ptEnd.header.stamp = cloud_time;
        ptStart.header.stamp = cloud_time;
        listener_2.transformPoint("odom", ptStart, ptStart_trans);
        listener_2.transformPoint("odom", ptEnd, ptEnd_trans);

        batteries.data.clear();
          
        batteries.data.push_back(ptStart_trans.point.x);
        batteries.data.push_back(ptStart_trans.point.y);
        batteries.data.push_back(ptEnd_trans.point.x);
        batteries.data.push_back(ptEnd_trans.point.y);

        batteries_publisher.publish(batteries);
      } catch(tf::TransformException ex) {
        ROS_ERROR("Unable to transform");
        continue;
      }
    }
  }

  void detectWallSegment()
  {
    std::vector<float> ranges = obstacles_found.ranges;
    std::vector<float> angles = obstacles_found.angles;

    float prev_xpos = 0;
    float prev_ypos = 0;

    float xpos = 0;
    float ypos = 0;
    float distance = 0;

    wall_segments.clear();

    std::vector<std::pair<float, float> > connected_points;

    for (int i = 0; i < ranges.size(); i++)
    {
      if (connected_points.size() < 1)
      {
        prev_xpos = cos(angles[i]) * ranges[i];
        prev_ypos = sin(angles[i]) * ranges[i];

        connected_points.push_back(std::make_pair(prev_xpos, prev_ypos));
        continue;
      }

      xpos = cos(angles[i]) * ranges[i];
      ypos = sin(angles[i]) * ranges[i];

      distance = sqrt(pow((prev_xpos - xpos), 2) + pow((prev_ypos - ypos), 2));

      if (distance < CONNECTION_THRESHOLD)
      {
        connected_points.push_back(std::make_pair(xpos, ypos));

        prev_xpos = xpos;
        prev_ypos = ypos;
      }
      else
      {
        std::pair<float, float> first_connection = connected_points.front();
        std::pair<float, float> last_connection = connected_points.back();

        float length_wall = sqrt(pow(first_connection.first - last_connection.first, 2) + pow(first_connection.second - last_connection.second, 2));
        if(length_wall > WALL_LENGTH_THRESHOLD) {
          wall_segments.push_back(make_pair(first_connection, last_connection));
          connected_points.clear();
        }
      }
    }

    if(connected_points.size() > 0) {
      std::pair<float, float> first_connection = connected_points.front();
      std::pair<float, float> last_connection = connected_points.back();
      float length_wall = sqrt(pow(first_connection.first - last_connection.first, 2) + pow(first_connection.second - last_connection.second, 2));

      if(length_wall > WALL_LENGTH_THRESHOLD) {
        wall_segments.push_back(make_pair(first_connection, last_connection));
        connected_points.clear();
      }
    } 
  }

  void visualizeWalls()
  {
    visualization_msgs::MarkerArray obstacle_walls;
    visualization_msgs::Marker wall;

    wall.header.frame_id = "/base_link";
    wall.header.stamp = cloud_time;

    wall.ns = "obstacle_walls";
    wall.type = visualization_msgs::Marker::CUBE;
    wall.action = visualization_msgs::Marker::ADD;

    wall.pose.position.z = 0.1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    //marker.scale.x = 1.0;
    wall.scale.y = 0.01;
    wall.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    wall.color.r = 0.0f;
    wall.color.g = 1.0f;
    wall.color.b = 0.0f;
    wall.color.a = 1.0;

    int id = 0;

    for (int i = 0; i < wall_segments.size(); i++)
    {
      float x1 = wall_segments[i].first.first;
      float y1 = wall_segments[i].first.second;
      float x2 = wall_segments[i].second.first;
      float y2 = wall_segments[i].second.second;

      float centre_x = (x1 + x2) / 2;
      float centre_y = (y1 + y2) / 2;

      wall.pose.position.x = centre_x;
      wall.pose.position.y = centre_y;
      
      float rotation = atan2((y2 - y1), (x2 - x1));
      float length = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));

      wall.scale.x = length;

      tf::Quaternion q;
      q.setRPY(0.0, 0.0, rotation);
      tf::quaternionTFToMsg(q, wall.pose.orientation);

      wall.id = id;
      id++;

      obstacle_walls.markers.push_back(wall);
    }

    obstacle_wall_pub.publish(obstacle_walls);
  }

private:

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

  // Create a ROS publisher for the output point cloud
  ros::Publisher pub = obs.n.advertise<sensor_msgs::PointCloud2>("obstacle/output", 1);

  // ROS publisher for obstacles found
  ros::Publisher obstacle_publisher = obs.n.advertise<project_msgs::depth>("/depth", 1);

  tf::TransformListener listener;

  int count = 0;
  while (obs.n.ok())
  {
    ros::spinOnce();

    if(obs.point_cloud_received) {
      // Transform cloud
      listener.lookupTransform("/base_link", "/camera_depth_optical_frame", ros::Time(0), obs.transform);
      pcl_ros::transformPointCloud("/camera_depth_optical_frame", obs.original_pc, obs.transformed_pc, listener);

      // Publish the data
      obs.removeUnwantedData();

      if(obs.PUBLISH_MARKERS)
      {
        obs.visualizeBins();

        if(obs.obstacles_found.ranges.size() > 1)
        {
          obs.detectWallSegment();
          obs.visualizeWalls();
        }
      }

      pub.publish(obs.transformed_pc);

      obstacle_publisher.publish(obs.obstacles_found);

      
      if(obs.angular_vel < obs.ANGULAR_VELOCITY_THRESHOLD) {
        obs.sendBatteries();
      }
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
