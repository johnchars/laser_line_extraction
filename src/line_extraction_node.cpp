#include "laser_line_extraction/line_extraction_ros.h"
#include <ros/console.h>

#include "std_srvs/SetBool.h"

namespace {

class Node {
 public:
  Node(ros::NodeHandle* nh_ptr,
       ros::NodeHandle* nh_local_ptr,
       tf2_ros::Buffer* tf_buffer);
  ~Node() {
    if (line_extraction_ptr_ != nullptr) {
      delete line_extraction_ptr_;
      line_extraction_ptr_ = nullptr;
    }
  }
 private:
  bool HandleEnableRequest(std_srvs::SetBool::Request& request,
                           std_srvs::SetBool::Response& response);

  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;
  tf2_ros::Buffer* tf_buffer_;
  line_extraction::LineExtractionROS* line_extraction_ptr_;
  ros::ServiceServer enable_service_;
};

Node::Node(ros::NodeHandle* nh_ptr,
           ros::NodeHandle* nh_local_ptr,
           tf2_ros::Buffer* tf_buffer) :
    nh_ptr_(nh_ptr), nh_local_ptr_(nh_local_ptr),
    tf_buffer_(tf_buffer), line_extraction_ptr_(nullptr) {
  enable_service_ = nh_ptr->advertiseService(
    "intra_detect_angle", &Node::HandleEnableRequest, this);
}

bool Node::HandleEnableRequest(std_srvs::SetBool::Request& request,
                               std_srvs::SetBool::Response& response) {
  if (request.data) {
    ROS_INFO_STREAM("Enable line detector.");
    if (line_extraction_ptr_ != nullptr) {
      delete line_extraction_ptr_;
      line_extraction_ptr_ = nullptr;
    }
    line_extraction_ptr_ = new line_extraction::LineExtractionROS(
        *nh_ptr_, *nh_local_ptr_, tf_buffer_);
    response.success = true;
    response.message = "Enable";
  } else {
    ROS_INFO_STREAM("Disable line detector.");
    if (line_extraction_ptr_ != nullptr) {
      delete line_extraction_ptr_;
      line_extraction_ptr_ = nullptr;
    }
    response.success = true;
    response.message = "Disable";
  }
  return true;
}

} // namespace

int main(int argc, char **argv)
{

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug))
  {
     ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("Starting line_extraction_node.");

  ros::init(argc, argv, "line_extraction_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  // ros::ServiceServer enable_service = nh.advertiseService("detect_angle", );
  // line_extraction::LineExtractionROS line_extractor(nh, nh_local);
  Node node(&nh, &nh_local, &tf_buffer);
  // Use 2 threads
  // ros::AsyncSpinner spinner(2);
  // spinner.start();
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
