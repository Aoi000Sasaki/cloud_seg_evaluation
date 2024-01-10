#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <algorithm>
#include <pcl/point_types.h>

#include <cloud_seg_evaluation/cloud_seg_evaluation.h>

namespace cloud_seg_evaluation
{
float isTimestampMatched = 1.0e-9;
float isPointMatched = 3.0e-5;
std::map<std::string, int[]> label_color_map = {
  {"unlabeled", {0, 0, 0}},
  {"outlier", {0, 0, 0}},
  {"animal", {64, 128, 64}},
  {"curb", {192, 0, 128}},
  {"fence", {64, 64, 128}},
  {"guardrail", {64, 64, 64}},
  {"wall", {128, 0, 192}},
  {"bike lane", {192, 128, 64}},
  {"crosswalk - plain", {128, 64, 64}},
  {"curb cut", {0, 0, 192}},
  {"parking", {64, 192, 128}},
  {"pedestrian area", {64, 64, 0}},
  {"rail track", {128, 128, 64}},
  {"road", {128, 64, 128}},
  {"service lane", {192, 192, 128}},
  {"sidewalk", {0, 0, 64}},
  {"bridge", {0, 64, 64}},
  {"building", {192, 128, 128}},
  {"tunnel", {64, 192, 0}},
  {"person", {0, 0, 192}},
  {"bicyclist", {128, 128, 192}},
  {"motorcyclist", {192, 128, 192}},
  {"other rider", {64, 0, 64}},
  {"lane marking - crosswalk", {192, 0, 64}},
  {"lane marking - general", {128, 128, 0}},
  {"mountain", {192, 0, 192}},
  {"sand", {64, 192, 0}},
  {"sky", {0, 128, 192}},
  {"snow", {192, 128, 64}},
  {"terrain", {128, 192, 192}},
  {"vegetation", {64, 64, 192}},
  {"water", {0, 192, 128}},
  {"banner", {192, 128, 64}},
  {"bench", {64, 64, 128}},
  {"bike rack", {64, 128, 192}},
};

CloudSegEvaluation::CloudSegEvaluation()
  : pnh_("~")
  , sub_correct_cloud_(nh_, "correct_cloud", 10)
  , sub_my_cloud_(nh_, "my_cloud", 10)
  , sync_(SyncPolicy(10), sub_correct_cloud_, sub_my_cloud_)
  , tfl_(tf_)
{
  pub_debug_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("debug_cloud", 1);
  sync_.registerCallback(boost::bind(&CloudSegEvaluation::sync_callback, this, _1, _2));
}

CloudSegEvaluation::~CloudSegEvaluation()
{
}

void CloudSegEvaluation::sync_callback(const sensor_msgs::PointCloud2ConstPtr& correct_cloud_msg,
                                          const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg)
{
  if (correct_cloud_msg->header.stamp.toSec() - my_cloud_msg->header.stamp.toSec() < isTimestampMatched) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_correct_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_my_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*correct_cloud_msg, *pcl_correct_cloud);
    pcl::fromROSMsg(*my_cloud_msg, *pcl_my_cloud);

    // Transform the point cloud to the same frame
    pcl_ros::transformPointCloud(correct_cloud_msg->header.frame_id, *pcl_my_cloud, *pcl_my_cloud, tf_);

    CloudSegEvaluation::checkLabelConsistency(pcl_correct_cloud, pcl_my_cloud);
  }
}

void CloudSegEvaluation::checkLabelConsistency(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& correct_cloud,
                                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& my_cloud)
{
  // Sort the point cloud based on the X coordinate
  std::sort(correct_cloud->points.begin(), correct_cloud->points.end(),
            [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
              return p1.x > p2.x;
            });
  std::sort(my_cloud->points.begin(), my_cloud->points.end(),
            [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
              return p1.x > p2.x;
            });


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr correct_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  int max_iter = std::min(correct_cloud->points.size(), my_cloud->points.size());
  int match = 0;
  int unmatch = 0;
  for (int i = 0; i < max_iter; i++) {
    const pcl::PointXYZRGB& cr_pt = correct_cloud->points[i];
    const pcl::PointXYZRGB& my_pt = my_cloud->points[i];
    float diff = abs(cr_pt.x - my_pt.x) + abs(cr_pt.y - my_pt.y) + abs(cr_pt.z - my_pt.z);
    if (diff < isPointMatched) {
      if (cr_pt.r == my_pt.r && cr_pt.g == my_pt.g && cr_pt.b == my_pt.b) {
        correct_cloud_filtered->points.push_back(cr_pt);
      }
      match++;
    } else {
      unmatch++;
    }
  }

  // ROS_INFO("match: %d, unmatch: %d", match, unmatch);
  // ROS_INFO("correct_cloud_filtered size: %d", correct_cloud_filtered->points.size());

  const sensor_msgs::PointCloud2Ptr correct_cloud_filtered_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*correct_cloud_filtered, *correct_cloud_filtered_msg);
  correct_cloud_filtered_msg->header.frame_id = correct_cloud->header.frame_id;
  
  pub_debug_cloud_.publish(correct_cloud_filtered_msg);
}
}  // namespace cloud_seg_evaluation

int main(int arg, char** argv)
{
  ros::init(arg, argv, "cloud_seg_evaluation");
  cloud_seg_evaluation::CloudSegEvaluation node;
  ros::spin();
  return 0;
}
