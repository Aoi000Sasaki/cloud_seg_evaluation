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
#include <chrono>
#include <ctime>
#include <iomanip>

#include <cloud_seg_evaluation/cloud_seg_evaluation.h>

namespace cloud_seg_evaluation
{
CloudSegEvaluation::CloudSegEvaluation()
  : pnh_("~")
  , sub_correct_cloud_(nh_, "correct_cloud", 10)
  , sub_my_cloud_(nh_, "my_cloud", 10)
  , sync_(SyncPolicy(10), sub_correct_cloud_, sub_my_cloud_)
  , tfl_(tf_)
{
  pub_debug_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("positive_cloud", 1);
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
    CloudSegEvaluation::checkLabelConsistency(correct_cloud_msg, my_cloud_msg);
  }
}

void CloudSegEvaluation::checkLabelConsistency(const sensor_msgs::PointCloud2ConstPtr& correct_cloud_msg,
                                               const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_correct_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_my_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*correct_cloud_msg, *pcl_correct_cloud);
  pcl::fromROSMsg(*my_cloud_msg, *pcl_my_cloud);

  // Transform the point cloud to the same frame
  pcl_ros::transformPointCloud(correct_cloud_msg->header.frame_id, *pcl_my_cloud, *pcl_my_cloud, tf_);

  // Sort the point cloud based on the X coordinate
  std::sort(pcl_correct_cloud->points.begin(), pcl_correct_cloud->points.end(),
            [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
              return p1.x > p2.x;
            });
  std::sort(pcl_my_cloud->points.begin(), pcl_my_cloud->points.end(),
            [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
              return p1.x > p2.x;
            });

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr correct_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  int max_iter = std::min(pcl_correct_cloud->points.size(), pcl_my_cloud->points.size());
  bool is_unmatch_printed = false;

  for (auto& label_color : label_color_map) {
    evaluation eval = {correct_cloud_msg->header.stamp.toSec(), 0, 0, label_color.first, 0, 0, 0, 0, 0};

    int pointMatch = 0;
    int pointUnmatch = 0;
    bool is_ignore = false;
    for (int i = 0; i < max_iter; i++) {
      const pcl::PointXYZRGB& cr_pt = pcl_correct_cloud->points[i];
      const pcl::PointXYZRGB& my_pt = pcl_my_cloud->points[i];
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

        // if true: cr_pt represent the label_color.first(=label)
        if (label_color.second[0][0] == cr_pt.r && label_color.second[0][1] == cr_pt.g && label_color.second[0][2] == cr_pt.b) {
          // if true: cr_pt and my_pt represent the same label
          if (label_color.second[1][0] == my_pt.r && label_color.second[1][1] == my_pt.g && label_color.second[1][2] == my_pt.b) {
            correct_cloud_filtered->points.push_back(cr_pt);
            eval.positive++;
          } else {
            eval.false_negative++;
          }
        } else {
          // if true: my_pt represent the label_color.first(=label) but cr_pt does not
          if (label_color.second[1][0] == my_pt.r && label_color.second[1][1] == my_pt.g && label_color.second[1][2] == my_pt.b) {
            eval.false_positive++;
          } else {
            eval.negative++; // cr_pt and my_pt also do not represent the label_color.first(=label)
          }
        }        
      } else {
        pointUnmatch++;
      }
    }
    if (!is_unmatch_printed) {
      ROS_INFO("timestamp : %d\t unmatch/match points : %d/%d", eval.timestamp, pointUnmatch, pointMatch);
    }
    ROS_INFO("  %-15s positive: %-6d false_positive: %-6d false_negative: %-6d negative: %-6d ignore: %-7d",
             eval.label.c_str(), eval.positive, eval.false_positive, eval.false_negative, eval.negative, eval.ignore);
    eval.unmatch = pointUnmatch;
    eval.match = pointMatch;

    evaluation_vector.push_back(eval);
    is_unmatch_printed = true;
  }

  const sensor_msgs::PointCloud2Ptr correct_cloud_filtered_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*correct_cloud_filtered, *correct_cloud_filtered_msg);
  correct_cloud_filtered_msg->header.frame_id = correct_cloud_msg->header.frame_id;
  
  pub_debug_cloud_.publish(correct_cloud_filtered_msg);
  saveCloud(pcl_correct_cloud, pcl_my_cloud, correct_cloud_filtered, correct_cloud_msg->header.stamp.toSec());
}

std::vector<evaluation> CloudSegEvaluation::getEvaluationVector()
{
  return evaluation_vector;
}

void CloudSegEvaluation::saveCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_correct_cloud,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_my_cloud,
                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr correct_cloud_filtered,
                                   int timestamp)
{
  if (!is_log_dir_created) {
    printf("log dir is not created\n");
    return;
  }

  // Convert the color of pcl_my_cloud according to label_color_map
  for (auto& label_color : label_color_map) {
    for (int i = 0; i < pcl_my_cloud->points.size(); i++) {
      if (pcl_my_cloud->points[i].r == label_color.second[1][0] &&
          pcl_my_cloud->points[i].g == label_color.second[1][1] &&
          pcl_my_cloud->points[i].b == label_color.second[1][2]) {
        pcl_my_cloud->points[i].r = label_color.second[0][0];
        pcl_my_cloud->points[i].g = label_color.second[0][1];
        pcl_my_cloud->points[i].b = label_color.second[0][2];
      }
    }
  }
  
  std::string correct_cloud_file = log_dir + "/" + std::to_string(timestamp) + "-correct_cloud.pcd";
  std::string my_cloud_file = log_dir + "/" + std::to_string(timestamp) + "-my_cloud.pcd";
  std::string correct_cloud_filtered_file = log_dir + "/" + std::to_string(timestamp) + "-correct_cloud_filtered.pcd";
  pcl_correct_cloud->width = pcl_correct_cloud->points.size();
  pcl_my_cloud->width = pcl_my_cloud->points.size();
  correct_cloud_filtered->width = correct_cloud_filtered->points.size();
  pcl_correct_cloud->height = 1;
  pcl_my_cloud->height = 1;
  correct_cloud_filtered->height = 1;
  pcl::io::savePCDFileASCII(correct_cloud_file, *pcl_correct_cloud);
  pcl::io::savePCDFileASCII(my_cloud_file, *pcl_my_cloud);
  pcl::io::savePCDFileASCII(correct_cloud_filtered_file, *correct_cloud_filtered);
}

void makeLogDir()
{
  // Get current time
  auto now = std::chrono::system_clock::now();
  std::time_t current_time = std::chrono::system_clock::to_time_t(now);

  // Format current time as 'YYYY-MM-DD-HH:MM:SS'
  std::stringstream ss;
  ss << std::put_time(std::localtime(&current_time), "%Y-%m-%d-%H:%M:%S");
  std::string current_time_str = ss.str();
  log_dir = log_dir + current_time_str;

  if (mkdir(log_dir.c_str(), 0777) == -1) {
    printf("log dir cannot be created (maybe already exists)\n");
  } else {
    is_log_dir_created = true;
    printf("log dir (%s) created\n", log_dir.c_str());
  }
}

void writeEvaluationLog(std::vector<evaluation> evaluation_vector)
{
  if (!is_log_dir_created) {
    printf("log dir is not created\n");
    return;
  }

  std::string log_file = log_dir + "/evaluations.csv";
  std::ofstream ofs(log_file);
  if (!ofs) {
    printf("log file cannot be created\n");
    return;
  }

  ofs << "timestamp,unmatch,match,label,positive,false_positive,false_negative,negative,ignore" << std::endl;
  for (auto& eval : evaluation_vector) {
    ofs << eval.timestamp << "," << eval.unmatch << "," << eval.match << "," << eval.label << "," << eval.positive << ","
        << eval.false_positive << "," << eval.false_negative << "," << eval.negative << "," << eval.ignore << std::endl;
  }
  ofs.close();
  printf("log file (%s) created\n", log_file.c_str());
}
}  // namespace cloud_seg_evaluation

int main(int arg, char** argv)
{
  cloud_seg_evaluation::makeLogDir();
  ros::init(arg, argv, "cloud_seg_evaluation");
  cloud_seg_evaluation::CloudSegEvaluation node;

  while (ros::ok()) {
    ros::spinOnce();
  }

  cloud_seg_evaluation::writeEvaluationLog(node.getEvaluationVector());

  return 0;
}
