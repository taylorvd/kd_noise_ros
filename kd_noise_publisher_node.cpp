/*
#include <ros/ros.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert ROS PointCloud2 message to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // Add k-distribution noise to the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  //kdtree.setInputCloud(cloud);
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZ point = cloud->points[i];
   
    printf("before noise %f \n",point.x);
    point.x += (gamma()%10)*0.1;//sqrt(sqr_distance) * rand();//0.8 * (rand() / (RAND_MAX + 1.0f) - 0.5f);
    printf("after noise %f \n ", point.x);
    point.y += (rand()%10)*0.1;//sqrt(sqr_distance) *rand();//* 0.8 * (rand() / (RAND_MAX + 1.0f) - 0.5f);
    point.z += (rand()%10)*0.1;//sqrt(sqr_distance) *rand();//* 0.8 * (rand() / (RAND_MAX + 1.0f) - 0.5f);
    cloud_noise->points.push_back(point);
  }
  cloud_noise->width = cloud_noise->points.size();
  cloud_noise->height = 1;
  cloud_noise->is_dense = true;

  // Convert PCL PointCloud to ROS PointCloud2 message
  sensor_msgs::PointCloud2 cloud_noise_msg;
  pcl::toROSMsg(*cloud_noise, cloud_noise_msg);
  cloud_noise_msg.header.frame_id = "noisy_cloud";

  // Publish the point cloud with noise
  pub.publish(cloud_noise_msg);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "kd_noise_publisher_node");
  ros::NodeHandle nh;

  // Subscribe to the input point cloud topic
  ros::Subscriber sub = nh.subscribe("os1_cloud_node/points", 10, cloud_cb);

  // Advertise the output point cloud topic
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);

  // Spin
  ros::spin();
}
*/

#include <ros/ros.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>



ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert ROS PointCloud2 message to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  // Add k-distribution noise to the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise(new pcl::PointCloud<pcl::PointXYZ>);
    std::random_device rd;
    std::mt19937 gen(rd());
 // A gamma distribution with alpha=1, and beta=2
    // approximates an exponential distribution.
    std::gamma_distribution<> d(2,0.5);


  for (size_t i = 0; i < cloud->points.size(); ++i) {
    pcl::PointXYZ point = cloud->points[i];
 
    //printf("before noise %f \n",point.x);
    point.x += d(gen)*0.1;
    //printf("after noise %f \n ", point.x);
    point.y += d(gen)*0.1;
    point.z += d(gen)*0.1;
    cloud_noise->points.push_back(point);
  }
  cloud_noise->width = cloud_noise->points.size();
  cloud_noise->height = 1;
  cloud_noise->is_dense = true;




     //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*cloud_noise).size(); i++)
  {
    pcl::PointXYZ pt(cloud_noise->points[i].x, cloud_noise->points[i].y, cloud_noise->points[i].z);
    float zAvg = (rand() % 100)*0.01;

    if (zAvg < 0.98) // e.g. remove all pts below zAvg
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(cloud_noise);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_noise);






  // Convert PCL PointCloud to ROS PointCloud2 message
  sensor_msgs::PointCloud2 cloud_noise_msg;
  pcl::toROSMsg(*cloud_noise, cloud_noise_msg);
  cloud_noise_msg.header.frame_id = "noisy_cloud";

  // Publish the point cloud with noise
  pub.publish(cloud_noise_msg);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "kd_noise_publisher_node");
  ros::NodeHandle nh;

  // Subscribe to the input point cloud topic
  ros::Subscriber sub = nh.subscribe("os1_cloud_node/points", 10, cloud_cb);

  // Advertise the output point cloud topic
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);

  // Spin
  ros::spin();
}
