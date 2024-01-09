// Copyright 2023 amsl

#include <ros/ros.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
// #include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <cloud_seg_evaluation/cloud_seg_evaluation.h>

#include <std_msgs/String.h>

namespace cloud_seg_evaluation
{
CloudSegEvaluation::CloudSegEvaluation()
  : pnh_("~")
  , sub_correct_cloud_(nh_, "correct_cloud", 10)
  , sub_my_cloud_(nh_, "my_cloud", 10)
  , sync_(SyncPolicy(10), sub_correct_cloud_, sub_my_cloud_)
{
  pub_debug_msg = nh_.advertise<std_msgs::String>("debug_msg", 1);
  sync_.registerCallback(boost::bind(&CloudSegEvaluation::sync_callback, this, _1, _2));
}

CloudSegEvaluation::~CloudSegEvaluation()
{
}

void CloudSegEvaluation::sync_callback(const sensor_msgs::PointCloud2ConstPtr& correct_cloud_msg,
                                          const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg)
{
  pub_debug_msg.publish(std_msgs::String());
  // ROS_INFO("sync_callback");
  printf("crt :%lf my :%lf", correct_cloud_msg->header.stamp.toSec(), my_cloud_msg->header.stamp.toSec());
}

void CloudSegEvaluation::checkLabelConsistency(const sensor_msgs::PointCloud2ConstPtr& correct_cloud,
                                                   const sensor_msgs::PointCloud2ConstPtr& my_cloud)
{
  // // ポイントクラウドのデータを取得
  // pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud1;
  // pcl::fromROSMsg(*cloud1, pcl_cloud1);

  // pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud2;
  // pcl::fromROSMsg(*cloud2, pcl_cloud2);
}
}  // namespace cloud_seg_evaluation

int main(int arg, char** argv)
{
  ros::init(arg, argv, "cloud_seg_evaluation");
  cloud_seg_evaluation::CloudSegEvaluation node;
  ros::spin();
  return 0;
}
