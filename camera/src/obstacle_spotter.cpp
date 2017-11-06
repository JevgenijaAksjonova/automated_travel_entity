#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    
}
int main (int argc, char** argv){
    ros::init (argc,argv,"obstacle_spotter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/camera/depth/points",1,cloud_cb);
}