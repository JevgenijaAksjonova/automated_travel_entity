#include "ros/ros.h"

#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
using namespace std;

class ObstaclePublisher
{
  public:
    ros::NodeHandle n;

    ros::Publisher obstacle_publisher;

    ros::Subscriber point_cloud_subscriber;

    FilterPublisher(int frequency)
    {
        n = ros::NodeHandle("~");
        /*
        if(!n.getParam("/filter/particle_params/nr_particles",nr_particles)){
            ROS_ERROR("failed to detect parameter 1");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/nr_measurements",nr_measurements)){
            ROS_ERROR("failed to detect parameter 2");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/nr_random_particles",nr_random_particles)){
            ROS_ERROR("failed to detect parameter 3");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/random_particle_spread",random_particle_spread)){
            ROS_ERROR("failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/particle_params/gaussian_particle_noise_spread",gaussian_particle_noise_spread)){
            ROS_ERROR("failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }

        if(!n.getParam("/filter/particle_params/using_random_particles",using_random_particles)){
            ROS_ERROR("failed to detect parameter 4");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/odom_noise/k_D",k_D)){
            ROS_ERROR("failed to detect parameter 5");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/odom_noise/k_V",k_V)){
            ROS_ERROR("failed to detect parameter 6");
            exit(EXIT_FAILURE);
        }
        if(!n.getParam("/filter/odom_noise/k_W",k_W)){
            ROS_ERROR("failed to detect parameter 7");
            exit(EXIT_FAILURE);
        }
        */

        obstacle_publisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
        point_cloud_subscriber = n.subscribe("/camera/depth/points", 1, &ObstaclePublisher::pointCloudCallback, this);
        
        
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    {
        ROS_INFO("Got point cloud data!");

    }




  private:
    
    int temp = 0;
};

int main(int argc, char **argv)
{

    ROS_INFO("Spin!");

    float frequency = 10;
    
    ros::init(argc, argv, "obstacle_detection_publisher");


    ObstaclePublisher obstacle(frequency);

    ros::Rate loop_rate(frequency);


    int count = 0;
    while (obstacle.n.ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
