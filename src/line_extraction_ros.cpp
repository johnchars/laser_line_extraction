#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>
#include <ros/console.h>
#include <std_msgs/Float64.h>


namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(ros::NodeHandle& nh,
                                     ros::NodeHandle& nh_local,
                                     tf2_ros::Buffer* tf_buffer) :
    nh_(nh),
    nh_local_(nh_local),
    tf_buffer_(tf_buffer) {
  loadParameters();
  line_1_publisher_ = nh_.advertise<
      laser_line_extraction::LineSegmentList>("line_1_segments", 1, true);
  line_2_publisher_ = nh_.advertise<
      laser_line_extraction::LineSegmentList>("line_2_segments", 1, true);
  scan_1_subscriber_ = nh_.subscribe(
      scan_1_topic_, 1, &LineExtractionROS::LaserScan1Callback, this);
  scan_2_subscriber_ = nh_.subscribe(
      scan_2_topic_, 1, &LineExtractionROS::LaserScan2Callback, this);
  angle_publisher_ = nh_.advertise<std_msgs::Float64>(
      "intra_detected_angle", 1, true);
  if (pub_markers_)
  {
    marker_1_publisher_ = nh_.advertise<
        visualization_msgs::Marker>("line_1_markers", 1, true);
    marker_2_publisher_ = nh_.advertise<
        visualization_msgs::Marker>("line_2_markers", 1, true);
  }
  timer_ = nh_.createWallTimer(ros::WallDuration(0.05),
                               &LineExtractionROS::TimerCallback, this);
}

LineExtractionROS::~LineExtractionROS() {}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::TimerCallback(const ros::WallTimerEvent& event) {
  if (!initialized_1_ || !initialized_2_)
    return;
  auto TransformStampedToTransform = [](
      const geometry_msgs::TransformStamped& lhs,
      T& rhs) {
    rhs.t = Eigen::Vector3d(
      lhs.transform.translation.x,
      lhs.transform.translation.y,
      lhs.transform.translation.z);
    rhs.R = Eigen::Quaterniond(
      lhs.transform.rotation.w,
      lhs.transform.rotation.x,
      lhs.transform.rotation.y,
      lhs.transform.rotation.z);
  };

  if (!tf1_flag) {
    try {
      geometry_msgs::TransformStamped laser1_to_target =
        tf_buffer_->lookupTransform(target_frame_id_, frame_id_1_,
                                    ros::Time(0), ros::Duration(0.5));
      T T_laser2target;
      TransformStampedToTransform(laser1_to_target, T_laser2target);
      R_1_ = T_laser2target.R;
      tf1_flag = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN("tf from %s to %s over time: %s", frame_id_1_.c_str(),
                                                 target_frame_id_.c_str(),
                                                 ex.what());
      return;
    }
  }
  if (!tf2_flag) {
    try {
      geometry_msgs::TransformStamped laser2_to_target =
        tf_buffer_->lookupTransform(target_frame_id_, frame_id_2_,
                                    ros::Time(0), ros::Duration(0.5));
      T T_laser2target;
      TransformStampedToTransform(laser2_to_target, T_laser2target);
      R_2_ = T_laser2target.R;
      tf2_flag = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN("tf from %s to %s over time: %s", frame_id_2_.c_str(),
                                                 target_frame_id_.c_str(),
                                                 ex.what());
      return;
    }
  }

  if (!tf1_flag || !tf2_flag || !enable_detector_) {
    return;
  }

  // Extract the lines
  std::vector<Line> lines;
  {
    // const std::lock_guard<std::mutex> lock(mutex_1_);
    line_extraction_1_.extractLines(lines);
  }
  // Populate message
  laser_line_extraction::LineSegmentList msg;
  populateLineSegListMsg(lines, msg, frame_id_1_);

  // Publish the lines
  line_1_publisher_.publish(msg);

  std::vector<Line> lines_2;
  {
    // const std::lock_guard<std::mutex> lock(mutex_2_);
    line_extraction_2_.extractLines(lines_2);
  }
  laser_line_extraction::LineSegmentList msg2;
  populateLineSegListMsg(lines_2, msg2, frame_id_2_);

  line_2_publisher_.publish(msg2);

  // Also publish markers if parameter publish_markers is set to true
  if (pub_markers_)
  {
    visualization_msgs::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg, frame_id_1_);
    marker_1_publisher_.publish(marker_msg);

    visualization_msgs::Marker marker_msg2;
    populateMarkerMsg(lines_2, marker_msg2, frame_id_2_);
    marker_2_publisher_.publish(marker_msg2);
  }

  if (lines.empty() && lines_2.empty()) {
    ROS_WARN_STREAM("No line detected.");
    return;
  }

  double theta = .0;
  // lines.insert(lines.end(), lines_2.begin(), lines_2.end());
  // double angle = .0;
  if (ExtractAngleFromLines(lines, R_1_, lines_2, R_2_, theta)) {
    // double line_angle = theta + kPi / 2;
    // angle = std::atan2(std::sin(line_angle), std::cos(line_angle));
  } else {
    ROS_WARN_STREAM("Line feature lack of representativeness.");
    // angle = .0;
  }
  auto Average = [](const std::vector<double>& angles) ->double {
    double sum = .0;
    for (size_t i = 0; i < angles.size(); ++i) {
      sum += angles[i];
    }
    return sum / angles.size();
  };

  if (theta != .0 && angles_.empty()) {
    angles_.emplace_back(theta);
  } else if (std::abs(theta - Average(angles_)) < threshold) {
    angles_.emplace_back(theta);
  } else {
    angles_.clear();
    // detect failed.
  }

  if (angles_.size() > 9 && enable_detector_) {
    const double angle = Average(angles_);
    ROS_DEBUG_STREAM("Detected good line: " << angle);
    std_msgs::Float64 msg;
    msg.data = angle;
    angle_publisher_.publish(msg);
    enable_detector_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{

  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node

  std::string scan_1_topic, scan_2_topic;
  bool pub_markers;

  std::string target_frame;
  nh_local_.param<std::string>("target_frame", target_frame, "base_link");
  target_frame_id_ = target_frame;
  ROS_DEBUG("target_frame_id: %s", target_frame.c_str());

  nh_local_.param<std::string>("scan_1_topic", scan_1_topic, "scan1");
  scan_1_topic_ = scan_1_topic;
  ROS_DEBUG("scan_1_topic: %s", scan_1_topic_.c_str());

  nh_local_.param<std::string>("scan_2_topic", scan_2_topic, "scan2");
  scan_2_topic_ = scan_2_topic;
  ROS_DEBUG("scan_2_topic: %s", scan_2_topic_.c_str());

  nh_local_.param<bool>("publish_markers", pub_markers, false);
  pub_markers_ = pub_markers;
  ROS_DEBUG("publish_markers: %s", pub_markers ? "true" : "false");

  // Parameters used by the line extraction algorithm

  double bearing_std_dev, range_std_dev, least_sq_angle_thresh,
         least_sq_radius_thresh, max_line_gap, min_line_length,
         min_range, max_range, min_split_dist, outlier_dist;
  int min_line_points;

  nh_local_.param<double>("bearing_std_dev", bearing_std_dev, 1e-3);
  line_extraction_1_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  line_extraction_2_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

  nh_local_.param<double>("range_std_dev", range_std_dev, 0.02);
  line_extraction_1_.setRangeVariance(range_std_dev * range_std_dev);
  line_extraction_2_.setRangeVariance(range_std_dev * range_std_dev);
  ROS_DEBUG("range_std_dev: %f", range_std_dev);

  nh_local_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
  line_extraction_1_.setLeastSqAngleThresh(least_sq_angle_thresh);
  line_extraction_2_.setLeastSqAngleThresh(least_sq_angle_thresh);
  ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);

  nh_local_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
  line_extraction_1_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  line_extraction_2_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

  nh_local_.param<double>("max_line_gap", max_line_gap, 0.4);
  line_extraction_1_.setMaxLineGap(max_line_gap);
  line_extraction_2_.setMaxLineGap(max_line_gap);
  ROS_DEBUG("max_line_gap: %f", max_line_gap);

  nh_local_.param<double>("min_line_length", min_line_length, 0.5);
  line_extraction_1_.setMinLineLength(min_line_length);
  line_extraction_2_.setMinLineLength(min_line_length);
  ROS_DEBUG("min_line_length: %f", min_line_length);

  nh_local_.param<double>("min_range", min_range, 0.4);
  line_extraction_1_.setMinRange(min_range);
  line_extraction_2_.setMinRange(min_range);
  ROS_DEBUG("min_range: %f", min_range);

  nh_local_.param<double>("max_range", max_range, 30.0);
  line_extraction_1_.setMaxRange(max_range);
  line_extraction_2_.setMaxRange(max_range);
  ROS_DEBUG("max_range: %f", max_range);

  nh_local_.param<double>("min_split_dist", min_split_dist, 0.05);
  line_extraction_1_.setMinSplitDist(min_split_dist);
  line_extraction_2_.setMinSplitDist(min_split_dist);
  ROS_DEBUG("min_split_dist: %f", min_split_dist);

  nh_local_.param<double>("outlier_dist", outlier_dist, 0.05);
  line_extraction_1_.setOutlierDist(outlier_dist);
  line_extraction_2_.setOutlierDist(outlier_dist);
  ROS_DEBUG("outlier_dist: %f", outlier_dist);

  nh_local_.param<int>("min_line_points", min_line_points, 9);
  line_extraction_1_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  line_extraction_2_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  ROS_DEBUG("min_line_points: %d", min_line_points);

  ROS_DEBUG("*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(
    const std::vector<Line> &lines,
    laser_line_extraction::LineSegmentList &line_list_msg,
    const std::string& frame_id)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    laser_line_extraction::LineSegment line_msg;
    line_msg.angle = cit->getAngle();
    line_msg.radius = cit->getRadius();
    line_msg.covariance = cit->getCovariance();
    line_msg.start = cit->getStart();
    line_msg.end = cit->getEnd();
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id;
  line_list_msg.header.stamp = ros::Time::now();
}

void LineExtractionROS::populateMarkerMsg(
    const std::vector<Line> &lines,
    visualization_msgs::Marker &marker_msg,
    const std::string& frame_id)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = ros::Time::now();
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_1_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");
}

void LineExtractionROS::CacheData(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg,
    LineExtraction& line_extraction) {
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i) {
    // theta of pho
    const double bearing = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(bearing);
    cos_bearings.push_back(cos(bearing));
    sin_bearings.push_back(sin(bearing));
    indices.push_back(i);
  }

  line_extraction.setCachedData(bearings, cos_bearings, sin_bearings, indices);
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::LaserScan1Callback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
  if (!enable_detector_)
    return;

  if (!initialized_1_) {
    frame_id_1_ = scan_msg->header.frame_id;
    CacheData(scan_msg, line_extraction_1_);
    ROS_DEBUG("Data 1 has been cached.");
    initialized_1_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(),
                                          scan_msg->ranges.end());

  // const std::lock_guard<std::mutex> lock(mutex_1_);
  line_extraction_1_.setRangeData(scan_ranges_doubles);
}

void LineExtractionROS::LaserScan2Callback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
  if (!enable_detector_)
    return;
  if (!initialized_2_) {
    frame_id_2_ = scan_msg->header.frame_id;
    CacheData(scan_msg, line_extraction_2_);
    ROS_DEBUG("Data 2 has been cached.");
    initialized_2_ = true;
  }
  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(),
                                          scan_msg->ranges.end());

  // const std::lock_guard<std::mutex> lock(mutex_2_);
  line_extraction_2_.setRangeData(scan_ranges_doubles);
}

bool LineExtractionROS::ExtractAngleFromLines(
    const std::vector<Line>& lines_1,
    const Eigen::Quaterniond& R1,
    const std::vector<Line>& lines_2,
    const Eigen::Quaterniond& R2, double& theta) {
  if (lines_1.empty() && lines_2.empty())
    return false;
  // ROS_DEBUG_STREAM("line1's size: " << lines_1.size()
  //     << ", lines2's size: " << lines_2.size());
  std::vector<LineProperty> lines_property;
  // std::cout << "Angle list: \n";
  for (const auto& line : lines_1) {
    // 从圆切线角度转成直线角度
    double angle = NormalizeAngle(line.getAngle() + kPi/2);
    // 归一化
    angle = std::atan2(std::sin(angle), std::cos(angle));
    double length = line.length();
    Eigen::Quaterniond q(std::cos(angle/2), .0, .0, std::sin(angle/2));
    Eigen::Quaterniond result_q = R_1_ * q;
    angle = NormalizeAngle(2 * std::atan2(result_q.z(), result_q.w()));
    // std::cout << "1. " << angle << "\t";
    lines_property.emplace_back(LineProperty{angle, length});
  }
  // std::cout << "\n";
  for (const auto& line : lines_2) {
    // 从圆切线角度转成直线角度
    double angle = NormalizeAngle(line.getAngle() + kPi/2);
    // 归一化

    double length = line.length();
    Eigen::Quaterniond q(std::cos(angle/2), .0, .0, std::sin(angle/2));
    Eigen::Quaterniond result_q = R_2_ * q;
    angle = NormalizeAngle(2 * std::atan2(result_q.z(), result_q.w()));
    // std::cout << "2. " << angle << "\t";
    lines_property.emplace_back(LineProperty{angle, length});
  }
  // std::cout << "\nangle list finished.\n";
  // ROS_DEBUG_STREAM("lines.size(): " << lines_property.size());
  typedef std::vector<size_t> Cluster;
  // 1 degree
  const double theta_step = 1.0 / 360.0 * kPi;
  const size_t grid_num = size_t(std::ceil(kPi / theta_step));
  Cluster clusters[grid_num];
  for (size_t i = 0; i < lines_property.size(); ++i) {
    double angle = lines_property[i].angle;
    // 对应360个间隔中的一个
    size_t index = size_t(std::floor(angle / theta_step));
    // std::cout << "index: " << index << ", angle " << angle <<'\n';
    if (index < 0 || index >= grid_num) {
      ROS_ERROR_STREAM("Index overbound: " << index << ", angle: "
        << angle << ", grid_num: " << grid_num);
    }
    clusters[index].emplace_back(i);
  }
  // <size of line, index in array>
  struct Element {
    size_t index;
    size_t size;
  };
  std::vector<Element> sorted_clusters;
  // std::map<size_t, size_t> sorted_clusters;
  for (size_t j = 0; j < grid_num; ++j) {
    if (clusters[j].empty())
      continue;
    sorted_clusters.emplace_back(Element{j, clusters[j].size()});
  }
  if (sorted_clusters.empty()) {
    ROS_ERROR_STREAM("Clusters is empty.");
    return false;
  }
  std::sort(sorted_clusters.begin(), sorted_clusters.end(),
            [](const Element& lhs, const Element& rhs) ->bool {
    return lhs.size > rhs.size;
  });
  // std::cout << "Start......\n";
  // for (const auto& element : sorted_clusters) {
  //   const size_t index = element.index;
  //   ROS_DEBUG_STREAM("Size: " << clusters[index].size() << ", angle: "
  //     << lines_property[clusters[index][0]].angle);
  // }
  // std::cout << "End\n";
  if (sorted_clusters.size() == 1) {
    // Include two lines
    const size_t index = sorted_clusters.begin()->index;
    double sum_angle = .0;
    for (size_t i = 0; i < clusters[index].size(); ++i) {
      double angle = lines_property[clusters[index][i]].angle;
      sum_angle += angle;
    }
    // average angle
    theta = sum_angle / clusters[index].size();
    return true;
  }
  auto IsSameCluster = [&](
      const std::vector<size_t>& first_cluster,
      const std::vector<size_t>& second_cluster,
      double& theta) ->bool {
    if (first_cluster.empty() || second_cluster.empty()) {
      assert(false);
      return false;
    }
    double delta_theta =
      std::abs(lines_property[first_cluster.front()].angle -
               lines_property[second_cluster.front()].angle);
    if (delta_theta < theta_step) {
      double sum_angle = .0;
      for (size_t i = 0; i < first_cluster.size(); ++i)
        sum_angle += lines_property[first_cluster[i]].angle;
      for (size_t j = 0; j < second_cluster.size(); ++j)
        sum_angle += lines_property[second_cluster[j]].angle;
      theta = sum_angle / (first_cluster.size() + second_cluster.size());
    } else if (delta_theta > (kPi / 2 - theta_step * 2) &&
               delta_theta < (kPi / 2 + theta_step * 2)) {
      // 两组直线相互垂直
      double sum_angle = .0;
      for (size_t i = 0; i < first_cluster.size(); ++i)
        sum_angle += lines_property[first_cluster[i]].angle;
      double compensate = .0;
      if (lines_property[first_cluster.front()].angle >
          lines_property[second_cluster.front()].angle) {
        compensate = kPi / 2.0;
      } else {
        compensate = - kPi / 2.0;
      }
      for (size_t j = 0; j < second_cluster.size(); ++j)
        sum_angle += lines_property[second_cluster[j]].angle + compensate;
      theta = sum_angle / (first_cluster.size() + second_cluster.size());
    } else {
      double sum_angle = .0;
      for (size_t i = 0; i < first_cluster.size(); ++i)
        sum_angle += lines_property[first_cluster[i]].angle;
      theta = sum_angle / first_cluster.size();
      return false;
    }
    return true;
  };
  if (sorted_clusters.size() == 2) {
    const auto& first_cluster = clusters[sorted_clusters.begin()->index];
    const auto& second_cluster = clusters[sorted_clusters.rbegin()->index];
    if (!IsSameCluster(first_cluster, second_cluster, theta)) {
      ROS_WARN_STREAM("Single line detected.");
    }
    return true;
  }

  auto it = sorted_clusters.begin();
  const auto& first_cluster = clusters[it->index];
  ++it;
  assert(it != sorted_clusters.end());
  const auto& second_cluster = clusters[it->index];
  ++it;
  assert(it != sorted_clusters.end());
  const auto& third_cluster = clusters[it->index];
  // merge similar lines
  if (IsSameCluster(first_cluster, second_cluster, theta)) {
    ROS_DEBUG_STREAM("first and second merged: " << first_cluster.size()
      << " and " << second_cluster.size());
  } else if (IsSameCluster(first_cluster, third_cluster, theta)) {
    ROS_DEBUG_STREAM("first and third merged: " << first_cluster.size()
      << " and " << third_cluster.size());
  } else if (IsSameCluster(second_cluster, third_cluster, theta)) {
    ROS_DEBUG_STREAM("Cross line over same line. first and second merged: "
      << first_cluster.size() << " and " << second_cluster.size());
  } else if (first_cluster.size() > 1) {
    ROS_DEBUG_STREAM("Only same cluster line found.");
    double sum_angle = .0;
    for (size_t i = 0; i < first_cluster.size(); ++i)
      sum_angle += lines_property[first_cluster[i]].angle;
    theta = sum_angle / first_cluster.size();
  } else {
    ROS_WARN_STREAM("Not enough line detected.");
    return false;
  }
  ROS_INFO_STREAM("Detected angle: " << theta);
  return true;
}

} // namespace line_extraction
