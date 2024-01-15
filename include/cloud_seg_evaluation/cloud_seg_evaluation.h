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
// label_color_map[label] = {{correct_cloud RGB}, {my_cloud RGB}}
std::map<std::string, std::array<std::array<int, 3>, 2>> label_color_map = {
// label                        correct_cloud     my_cloud
  {"Unlabeled",               {{{0  , 0  , 0  },  {0  , 0  , 0  }}}},
  {"Car",                     {{{100, 150, 245},  {0  , 0  , 142}}}},
  {"Bicycle",                 {{{100, 230, 245},  {119, 11 , 32 }}}},
  {"Bus",                     {{{100, 80 , 250},  {0  , 60 , 100}}}},
  {"Motorcycle",              {{{30 , 60 , 150},  {0  , 0  , 230}}}},
  {"Truck",                   {{{80 , 30 , 180},  {0  , 0  , 70 }}}},
  {"Person",                  {{{255, 30 , 30 },  {220, 20 , 60 }}}},
  {"Bicyclist",               {{{255, 40 , 200},  {255, 0  , 0  }}}},
  {"Road",                    {{{255, 0  , 255},  {128, 64 , 128}}}},
  {"Parking",                 {{{255, 150, 255},  {250, 170, 160}}}},
  {"Sidewalk",                {{{75 , 0  , 75 },  {244, 35 , 232}}}},
  {"Other-ground",            {{{175, 0  , 75 },  {81 , 0  , 81 }}}},
  {"Building",                {{{255, 200, 0  },  {70 , 70 , 70 }}}},
  {"Fence",                   {{{255, 120, 50 },  {190, 153, 153}}}},
  {"Vegetation",              {{{0  , 175, 0  },  {107, 142, 35 }}}},
  {"Terrain",                 {{{150, 240, 80 },  {152, 251, 152}}}},
  {"Pole",                    {{{255, 240, 150},  {153, 153, 153}}}},
  {"Traffic-sign",            {{{255, 0  , 0  },  {220, 220, 0  }}}},
};
// ignore color in my_cloud (behind the camera, out of sight)
std::vector<std::array<int, 3>> ignore_color = {
  {200, 0, 0},
  {0, 200, 0},
};
struct evaluation {
  int timestamp = 0;
  int unmatch = 0;
  int match = 0;
  std::string label;
  int positive = 0;
  int false_positive = 0;
  int false_negative = 0;
  int negative = 0;
  int ignore = 0;
};
class CloudSegEvaluation
{
public:
  CloudSegEvaluation();
  ~CloudSegEvaluation();

  void sync_callback(const sensor_msgs::PointCloud2ConstPtr& correct_cloud_msg,
                     const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg);
  void checkLabelConsistency(const sensor_msgs::PointCloud2ConstPtr& correct_cloud_msg,
                             const sensor_msgs::PointCloud2ConstPtr& my_cloud_msg);
  std::vector<evaluation> getEvaluationVector();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_debug_cloud_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_correct_cloud_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_my_cloud_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener tfl_;
  float isTimestampMatched;
  float isPointMatched;
  std::vector<evaluation> evaluation_vector;
};
std::string log_dir = "/home/user/ws/src/cloud_seg_evaluation/logs/";
bool is_log_dir_created = false;
void makeLogDir();
void writeEvaluationLog(std::vector<evaluation> evaluation_vector);
}  // namespace cloud_seg_evaluation

#endif  // CLOUD_SEG_EVALUATION_H