#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

ros::Subscriber sub;
ros::Publisher pub;
// std::set<int> color_set;
void coloredCloudCallback(const sensor_msgs::PointCloud2ConstPtr& colored_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr reColorized_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*colored_cloud, *cloud);
  pcl::fromROSMsg(*colored_cloud, *reColorized_cloud);

  pcl::PointXYZRGB& pt = cloud->points[0];
  // int color_int = 0;
  for (int i = 0; i < cloud->points.size(); i++)
  {
    pt = cloud->points[i];
    if (pt.r == 200 && pt.g == 0 && pt.b == 0)
    {
      reColorized_cloud->points[i].r = 180;
      reColorized_cloud->points[i].g = 180;
      reColorized_cloud->points[i].b = 180;
    }
    if (pt.r == 0 && pt.g == 200 && pt.b == 0)
    {
      reColorized_cloud->points[i].r = 0;
      reColorized_cloud->points[i].g = 0;
      reColorized_cloud->points[i].b = 0;
    }

    // color_int = pt.r * 1000000 + pt.g * 1000 + pt.b;
    // color_set.insert(color_int);
  }

  // Publish the reColorized_cloud
  const sensor_msgs::PointCloud2Ptr reColorized_cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*reColorized_cloud, *reColorized_cloud_msg);
  reColorized_cloud_msg->header.frame_id = colored_cloud->header.frame_id;
  pub.publish(reColorized_cloud_msg);


  // printf("---start---\n");
  // printf("color_set.size() = %d\n", color_set.size());
  // for (std::set<int>::iterator it = color_set.begin(); it != color_set.end(); it++)
  // {
  //   printf("%09d\n", *it);
  // }
  // printf("---end---\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reColorze");
  ros::NodeHandle nh;
  sub = nh.subscribe("/converted_cloud", 1, coloredCloudCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/reColorized_cloud", 1);

  ros::spin();

  return 0;
}
