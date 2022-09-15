#ifndef LINE_EXTRACTION_ROS_H
#define LINE_EXTRACTION_ROS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <mutex>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "tf2_ros/transform_listener.h"
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"

namespace line_extraction
{

constexpr double kPi = 3.14159265356;
const double threshold = 1.0 / 180.0 * kPi;

struct T {
  Eigen::Vector3d t;
  Eigen::Quaterniond R;
};

struct LineProperty {
  double angle;
  double length;
};

class LineExtractionROS
{

public:
  // Constructor / destructor
  LineExtractionROS(ros::NodeHandle&, ros::NodeHandle&, tf2_ros::Buffer*);
  ~LineExtractionROS();
  // Running
  void run();
  double GetAngle() { return angle_; }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::Subscriber scan_1_subscriber_;
  ros::Subscriber scan_2_subscriber_;
  ros::Publisher line_1_publisher_;
  ros::Publisher line_2_publisher_;
  ros::Publisher marker_1_publisher_;
  ros::Publisher marker_2_publisher_;
  ros::Publisher angle_publisher_;
  ros::WallTimer timer_;
  // Parameters
  std::string target_frame_id_;
  std::string frame_id_1_;
  std::string frame_id_2_;
  std::string scan_1_topic_;
  std::string scan_2_topic_;
  Eigen::Quaterniond R_1_;
  Eigen::Quaterniond R_2_;
  bool pub_markers_;
  // Line extraction
  LineExtraction line_extraction_1_;
  LineExtraction line_extraction_2_;

  double angle_ = .0;
  bool enable_detector_ = true;
  bool initialized_1_ = false;
  bool initialized_2_ = false;
  // 局部变量，因需要重置放入成员变量中
  bool tf1_flag = false;
  bool tf2_flag = false;

  std::vector<double> angles_;
  tf2_ros::Buffer* tf_buffer_;
  std::mutex mutex_1_;
  std::mutex mutex_2_;
  // Members
  void loadParameters();
  void populateLineSegListMsg(const std::vector<Line>&,
                              laser_line_extraction::LineSegmentList&,
                              const std::string&);

  void populateMarkerMsg(const std::vector<Line>&,
                         visualization_msgs::Marker&, const std::string&);

  void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
  void CacheData(const sensor_msgs::LaserScan::ConstPtr&, LineExtraction&);
  void LaserScan1Callback(const sensor_msgs::LaserScan::ConstPtr&);
  void LaserScan2Callback(const sensor_msgs::LaserScan::ConstPtr&);
  void TimerCallback(const ros::WallTimerEvent& event);

  bool ExtractAngleFromLines(
      const std::vector<Line>& lines_1,
      const Eigen::Quaterniond& R1,
      const std::vector<Line>& lines_2,
      const Eigen::Quaterniond& R2, double& theta);

  inline double NormalizeAngle(double angle) {
    while (angle < (-kPi))
      angle += 2 * kPi;
    while (angle > kPi)
      angle -= 2 * kPi;
    if (angle < 0)
      angle += kPi;
    return angle;
  }
};

} // namespace line_extraction

#endif
