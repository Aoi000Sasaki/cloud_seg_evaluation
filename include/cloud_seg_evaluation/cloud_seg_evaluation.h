#ifndef CLOUD_SEG_EVALUATION_H
#define CLOUD_SEG_EVALUATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>

namespace cloud_seg_evaluation
{
class CloudSegEvaluation
{
public:
  CloudSegEvaluation();
  ~CloudSegEvaluation();

  void sync_callback(const sensor_msgs::PointCloud2ConstPtr& correct_cloud_msg,
                     const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg);
  void checkLabelConsistency(const sensor_msgs::PointCloud2ConstPtr& correct_cloud,
                             const sensor_msgs::PointCloud2ConstPtr& my_cloud);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_debug_msg;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_correct_cloud_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_my_cloud_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;
};
}  // namespace cloud_seg_evaluation

#endif  // CLOUD_SEG_EVALUATION_H