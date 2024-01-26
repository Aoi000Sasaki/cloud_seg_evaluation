#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

std::set<int> color_set;
void coloredCloudCallback(const sensor_msgs::PointCloud2ConstPtr& colored_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*colored_cloud, *cloud);

  pcl::PointXYZRGB& pt = cloud->points[0];
  int color_int = 0;
  for (int i = 0; i < cloud->points.size(); i++)
  {
    pt = cloud->points[i];
    color_int = pt.r * 1000000 + pt.g * 1000 + pt.b;
    color_set.insert(color_int);
  }

  printf("---start---\n");
  printf("color_set.size() = %d\n", color_set.size());
  for (std::set<int>::iterator it = color_set.begin(); it != color_set.end(); it++)
  {
    printf("%09d\n", *it);
  }
  printf("---end---\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reColorze");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/colored_cloud", 1, coloredCloudCallback);

  ros::spin();

  return 0;
}
