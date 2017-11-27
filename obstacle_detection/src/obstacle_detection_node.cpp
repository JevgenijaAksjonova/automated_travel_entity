#include <ros/ros.h>
#include <tf/transform_listener.h>

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
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub;

    // Create a ROS publisher for the output point cloud
    ros::Publisher pub;

    // Sent vector for obstacle avoidance 
    std::vector< std::pair<float, float> > obstacles;


    ObstaclePublisher()
    {
        n = ros::NodeHandle("~");

        sub = n.subscribe ("/camera/depth/points", 1, &ObstaclePublisher::pointCloudCallback, this);

        pub = n.advertise<sensor_msgs::PointCloud2> ("obstacle/output", 1);

    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2 cloud_filtered;

      sensor_msgs::PointCloud2 transformed_pc;

      // Transform cloud
      listener.lookupTransform("/base_link", "/camera_depth_optical_frame", ros::Time(0), transform);
      pcl_ros::transformPointCloud("/camera_depth_optical_frame", *cloud_msg, transformed_pc, listener);

      /*
      // Convert to PCL data type
      pcl_conversions::toPCL(transformed_pc, *cloud);

      // Perform the actual filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(cloudPtr);
      sor.setLeafSize(0.1, 0.1, 0.1);
      sor.filter(cloud_filtered);

      // Convert to ROS data type
      sensor_msgs::PointCloud2 output;
      pcl_conversions::fromPCL(cloud_filtered, output);
      */

      // Publish the data
      pub.publish(transformed_pc);
    }

    /*
    void detectObstacles() 
    {

    }

    void removeUnwantedData()
    {
      
    }*/

  private:
    tf::TransformListener listener;
    tf::StampedTransform transform;
};


int main (int argc, char** argv)
{

  ROS_INFO("Spinning!");

  // Initialize ROS
  ros::init (argc, argv, "obstacle_detection");

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