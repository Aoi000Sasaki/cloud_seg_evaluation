#ifndef CLOUD_SEG_EVALUATION_H
#define CLOUD_SEG_EVALUATION_H

#include <ros/ros.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/Image.h>
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

  void sync_callback(const sensor_msgs::PointCloud2ConstPtr& kitti_cloud_msg,
                     const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg);
  bool checkLabelConsistency(const sensor_msgs::PointCloud2ConstPtr& cloud1,
                             const sensor_msgs::PointCloud2ConstPtr& cloud2);
  // {
  //   // ポイントクラウドのデータを取得
  //   pcl::PointCloud<pcl::PointXYZL> pcl_cloud1;
  //   pcl::fromROSMsg(*cloud1, pcl_cloud1);

  //   pcl::PointCloud<pcl::PointXYZL> pcl_cloud2;
  //   pcl::fromROSMsg(*cloud2, pcl_cloud2);

  //   // ラベルの一致判定
  //   for (size_t i = 0; i < pcl_cloud1.size(); ++i)
  //   {
  //     if (pcl_cloud1[i].label != pcl_cloud2[i].label)
  //     {
  //       // ラベルが一致しない場合
  //       return false;
  //     }
  //   }

  //   // 全ての点のラベルが一致する場合
  //   return true;
  // }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_colored_cloud_;
  ros::Publisher pub_debug_image_;
  ros::Subscriber sub_camera_info_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_kitti_cloud_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_my_cloud_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;
};
}  // namespace cloud_seg_evaluation

#endif  // CLOUD_SEG_EVALUATION_H