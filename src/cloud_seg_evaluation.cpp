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
// correct_cloudのラベルと色の対応
std::map<std::string, std::array<int, 3>> label_color_map = {
  {"car", {100, 150, 245}},
  {"aaa", {23, 3, 83}},
  {"bbb", {2, 93, 33}},
};
// my_cloudの中で無視する色（カメラ後方，視野外）
std::list<std::array<int, 3>> ignore_color = {
  {255, 0, 0},
  {0, 255, 0},
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
  isTimestampMatched = 1.0e-9;
  isPointMatched = 3.0e-5;
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
  std::list<evaluation> evaluation_list;
  int max_iter = std::min(correct_cloud->points.size(), my_cloud->points.size());
  bool is_unmatch_printed = false;

  for (auto& label_color : label_color_map) {
    evaluation eval = {label_color.first, 0, 0, 0, 0};

    int pointMatch = 0;
    int pointUnmatch = 0;
    bool is_ignore = false;
    for (int i = 0; i < max_iter; i++) {
      const pcl::PointXYZRGB& cr_pt = correct_cloud->points[i];
      const pcl::PointXYZRGB& my_pt = my_cloud->points[i];
      float diff = abs(cr_pt.x - my_pt.x) + abs(cr_pt.y - my_pt.y) + abs(cr_pt.z - my_pt.z);
      if (diff < isPointMatched) {
        pointMatch++;
        for (auto& ignore_color : ignore_color) {
          if (ignore_color[0] == my_pt.r && ignore_color[1] == my_pt.g && ignore_color[2] == my_pt.b) {
            eval.ignore++;
            is_ignore = true;
          }
        }
        if (is_ignore) {
          is_ignore = false;
          continue;
        }

        if (label_color.second[0] == cr_pt.r && label_color.second[1] == cr_pt.g && label_color.second[2] == cr_pt.b) {
          if (cr_pt.r == my_pt.r && cr_pt.g == my_pt.g && cr_pt.b == my_pt.b) {
            correct_cloud_filtered->points.push_back(cr_pt);
            eval.positive++;
          } else {
            eval.false_negative++;
          }
        } else {
          if (label_color.second[0] == my_pt.r && label_color.second[1] == my_pt.g && label_color.second[2] == my_pt.b) {
            eval.false_positive++;
          } else {
            eval.negative++;
          }
        }        
      } else {
        pointUnmatch++;
      }
    }
    if (!is_unmatch_printed) {
      ROS_INFO("--- unmatch/match points:  %d/%d ---", pointUnmatch, pointMatch);
    }
    ROS_INFO("label: %s, positive: %d, false_positive: %d, false_negative: %d, negative: %d, ignore: %d",
             eval.label.c_str(), eval.positive, eval.false_positive, eval.false_negative, eval.negative, eval.ignore);

    evaluation_list.push_back(eval);
    is_unmatch_printed = true;
  }

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
