
#include "rclcpp/rclcpp.hpp"

#include <unordered_map>
#include <unordered_set>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/aruco.hpp>
#include <memory>
#include <vector>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "pose_interfaces/msg/pose.hpp"

class ArucoLocalization : public rclcpp::Node {
public:
  ArucoLocalization();

private:
  void topic_callback(const sensor_msgs::msg::Image & msg);

  std::string world_name_;
  int count_world_marker_;
  std::unordered_set<int> ids_world_marker_;
  std::unordered_map<int, std::vector<cv::Point3f>> coords_world_marker_;

  //std::string robot_name_;
  //int count_robot_marker_;
  //std::unordered_set<int> ids_robot_marker_;
  //std::unordered_map<int, std::vector<cv::Point3f>> coords_robot_marker_;

  int count_block_;
  std::unordered_map<int, int> count_block_marker_;
  std::unordered_map<int, std::unordered_set<int>> ids_block_marker_;
  std::unordered_map<int, std::unordered_map<int, std::vector<cv::Point3f>>>
      coords_block_marker_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;                                                            
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::vector<rclcpp::Publisher<pose_interfaces::msg::Pose>::SharedPtr> block_coord_pub_;

  //std::unique_ptr<tf2_ros::TransformBroadcaster> tf_robot_;
  std::vector<std::unique_ptr<tf2_ros::TransformBroadcaster>> tf_block_;
  
  cv::Mat camMatrix_, distCoeffs_;
};
