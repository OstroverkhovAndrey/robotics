
#include "aruco_localization/aruco_localization_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

using std::placeholders::_1;

inline static bool readCameraParameters(std::string filename,
                                        cv::Mat &camMatrix,
                                        cv::Mat &distCoeffs) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["camera_matrix"] >> camMatrix;
  fs["distortion_coefficients"] >> distCoeffs;
  return true;
}

ArucoLocalization::ArucoLocalization() : Node("aruco_localization") {

  this->declare_parameter("world_name", "world");
  this->declare_parameter("count_world_marker", 6);
  for (int i = 0; i < this->get_parameter("count_world_marker").as_int(); ++i) {
    this->declare_parameter("id_world_marker_" + std::to_string(i), i);
    this->declare_parameter(
        "coord_world_marker_" + std::to_string(i),
        std::vector<double>{1, 2, 0, 2, 2, 0, 2, 1, 0, 1, 1, 0});
  }

  //this->declare_parameter("robot_name", "robot");
  //this->declare_parameter("count_robot_marker", 4);
  //for (int i = 0; i < this->get_parameter("count_robot_marker").as_int(); ++i) {
  //  this->declare_parameter("id_robot_marker_" + std::to_string(i), i);
  //  this->declare_parameter(
  //      "coord_robot_marker_" + std::to_string(i),
  //      std::vector<double>{1, 2, 0, 2, 2, 0, 2, 1, 0, 1, 1, 0});
  //}

  this->declare_parameter("count_block", 3);
  for (int i = 0; i < this->get_parameter("count_block").as_int(); ++i) {
    this->declare_parameter("count_block_" + std::to_string(i) + "_marker", 2);
    for (int j = 0;
         j < this->get_parameter("count_block_" + std::to_string(i) + "_marker")
                 .as_int();
         ++j) {
      this->declare_parameter(
          "id_block_" + std::to_string(i) + "_marker_" + std::to_string(j), i);
      this->declare_parameter(
          "coord_block_" + std::to_string(i) + "_marker_" + std::to_string(j),
          std::vector<double>{1, 2, 0, 2, 2, 0, 2, 1, 0, 1, 1, 0});
    }
  }

  world_name_ = this->get_parameter("world_name").as_string();
  count_world_marker_ = this->get_parameter("count_world_marker").as_int();
  for (int i = 0; i < count_world_marker_; ++i) {
    int marker_id =
        this->get_parameter("id_world_marker_" + std::to_string(i)).as_int();
    ids_world_marker_.insert(marker_id);
    coords_world_marker_[marker_id] = std::vector<cv::Point3f>(4);
    auto marker_coord =
        this->get_parameter("coord_world_marker_" + std::to_string(i))
            .as_double_array();
    coords_world_marker_[marker_id][0] =
        cv::Point3f(marker_coord[0], marker_coord[3], marker_coord[2]);
    coords_world_marker_[marker_id][1] =
        cv::Point3f(marker_coord[3], marker_coord[4], marker_coord[5]);
    coords_world_marker_[marker_id][2] =
        cv::Point3f(marker_coord[6], marker_coord[7], marker_coord[8]);
    coords_world_marker_[marker_id][3] =
        cv::Point3f(marker_coord[9], marker_coord[10], marker_coord[11]);
  }

  //robot_name_ = this->get_parameter("robot_name").as_string();
  //count_robot_marker_ = this->get_parameter("count_robot_marker").as_int();
  //for (int i = 0; i < count_robot_marker_; ++i) {
  //  int marker_id =
  //      this->get_parameter("id_robot_marker_" + std::to_string(i)).as_int();
  //  ids_robot_marker_.insert(marker_id);
  //  coords_robot_marker_[marker_id] = std::vector<cv::Point3f>(4);
  //  auto marker_coord =
  //      this->get_parameter("coord_robot_marker_" + std::to_string(i))
  //          .as_double_array();
  //  coords_robot_marker_[marker_id][0] =
  //      cv::Point3f(marker_coord[0], marker_coord[3], marker_coord[2]);
  //  coords_robot_marker_[marker_id][1] =
  //      cv::Point3f(marker_coord[3], marker_coord[4], marker_coord[5]);
  //  coords_robot_marker_[marker_id][2] =
  //      cv::Point3f(marker_coord[6], marker_coord[7], marker_coord[8]);
  //  coords_robot_marker_[marker_id][3] =
  //      cv::Point3f(marker_coord[9], marker_coord[10], marker_coord[11]);
  //}

  count_block_ = this->get_parameter("count_block").as_int();
  for (int i = 0; i < count_block_; ++i) {
    int count_marker =
        this->get_parameter("count_block_" + std::to_string(i) + "_marker")
            .as_int();
    count_block_marker_[i] = count_marker;
    ids_block_marker_[i] = std::unordered_set<int>();
    coords_block_marker_[i] =
        std::unordered_map<int, std::vector<cv::Point3f>>();
    for (int j = 0; j < count_marker; ++j) {
      int marker_id = this->get_parameter("id_block_" + std::to_string(i) +
                                          "_marker_" + std::to_string(j))
                          .as_int();
      ids_block_marker_[i].insert(marker_id);
      coords_block_marker_[i][marker_id] = std::vector<cv::Point3f>(4);
      auto marker_coord =
          this->get_parameter("coord_block_" + std::to_string(i) + "_marker_" +
                              std::to_string(j))
              .as_double_array();
      coords_block_marker_[i][marker_id][0] =
          cv::Point3f(marker_coord[0], marker_coord[3], marker_coord[2]);
      coords_block_marker_[i][marker_id][1] =
          cv::Point3f(marker_coord[3], marker_coord[4], marker_coord[5]);
      coords_block_marker_[i][marker_id][2] =
          cv::Point3f(marker_coord[6], marker_coord[7], marker_coord[8]);
      coords_block_marker_[i][marker_id][3] =
          cv::Point3f(marker_coord[9], marker_coord[10], marker_coord[11]);
    }
  }

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&ArucoLocalization::topic_callback, this, _1));
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "image_with_detect_markers", 10);
  for (int i = 0; i < count_block_; ++i) {
    block_coord_pub_.push_back(this->create_publisher<pose_interfaces::msg::Pose>(
      "block_coord"+std::to_string(i), 10));
  }
  //pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
  //    "tf_block0", 10);

  //tf_robot_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  for (int i = 0; i < count_block_; ++i) {
    tf_block_.push_back(std::make_unique<tf2_ros::TransformBroadcaster>(*this));
  }
  readCameraParameters("./src/aruco_localization/resources/cameraCalib.txt",
                       camMatrix_, distCoeffs_);
}

void ArucoLocalization::topic_callback(const sensor_msgs::msg::Image &msg) {

  cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cv::Mat imageCopy;
  image.copyTo(imageCopy);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  cv::aruco::detectMarkers(image, dictionary, corners, ids);

  if (ids.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
  }
  sensor_msgs::msg::Image::SharedPtr msg_pub =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageCopy)
          .toImageMsg();
  publisher_->publish(*msg_pub.get());

  std::vector<cv::Point3f> point_world;
  std::vector<cv::Point2f> pixel_world;

  //std::vector<cv::Point3f> point_robot;
  //std::vector<cv::Point2f> pixel_robot;

  std::vector<std::vector<cv::Point3f>> point_blocks(count_block_,
                                                    std::vector<cv::Point3f>());
  std::vector<std::vector<cv::Point2f>> pixel_blocks(count_block_,
                                                    std::vector<cv::Point2f>());

  for (int i = 0; i < (int)ids.size(); ++i) {
    int id = ids[i];
    if (ids_world_marker_.find(id) != ids_world_marker_.end()) {
      point_world.push_back(coords_world_marker_[id][0]);
      point_world.push_back(coords_world_marker_[id][1]);
      point_world.push_back(coords_world_marker_[id][2]);
      point_world.push_back(coords_world_marker_[id][3]);

      pixel_world.push_back(corners[i][0]);
      pixel_world.push_back(corners[i][1]);
      pixel_world.push_back(corners[i][2]);
      pixel_world.push_back(corners[i][3]);
    }
    //if (ids_robot_marker_.find(id) != ids_robot_marker_.end()) {
    //  point_robot.push_back(coords_robot_marker_[id][0]);
    //  point_robot.push_back(coords_robot_marker_[id][1]);
    //  point_robot.push_back(coords_robot_marker_[id][2]);
    //  point_robot.push_back(coords_robot_marker_[id][3]);

    //  pixel_robot.push_back(corners[i][0]);
    //  pixel_robot.push_back(corners[i][1]);
    //  pixel_robot.push_back(corners[i][2]);
    //  pixel_robot.push_back(corners[i][3]);
    //}
    for (int j = 0; j < count_block_; ++j) {
      if (ids_block_marker_[j].find(id) != ids_block_marker_[j].end()) {
        point_blocks[j].push_back(coords_block_marker_[j][id][0]);
        point_blocks[j].push_back(coords_block_marker_[j][id][1]);
        point_blocks[j].push_back(coords_block_marker_[j][id][2]);
        point_blocks[j].push_back(coords_block_marker_[j][id][3]);

        pixel_blocks[j].push_back(corners[i][0]);
        pixel_blocks[j].push_back(corners[i][1]);
        pixel_blocks[j].push_back(corners[i][2]);
        pixel_blocks[j].push_back(corners[i][3]);
      }
    }
  }

  // estimation world position
  cv::Mat rVectorWorld;
  cv::Mat tVectorWorld;
  try {
    cv::solvePnP(point_world, pixel_world, camMatrix_, distCoeffs_,
                 rVectorWorld, tVectorWorld);
  } catch (cv::Exception &exp) {
    RCLCPP_INFO(this->get_logger(), "Not 1 solvePnP!");
  }
  cv::Mat c_R_w;
  cv::Rodrigues(rVectorWorld, c_R_w);
  cv::Mat w_R_c = c_R_w.t();
  cv::Mat x = -w_R_c * tVectorWorld;
  cv::Vec3d w_P_wc = x;
  cv::Mat w_T_c =
      (cv::Mat_<double>(4, 4) << w_R_c.at<double>(0, 0), w_R_c.at<double>(0, 1),
       w_R_c.at<double>(0, 2), w_P_wc[0], w_R_c.at<double>(1, 0),
       w_R_c.at<double>(1, 1), w_R_c.at<double>(1, 2), w_P_wc[1],
       w_R_c.at<double>(2, 0), w_R_c.at<double>(2, 1), w_R_c.at<double>(2, 2),
       w_P_wc[2], 0, 0, 0, 1);

  // estimation robot position
  //{
  //  cv::Mat rVectorRobot;
  //  cv::Mat tVectorRobot;
  //  try {
  //    cv::solvePnP(point_robot, pixel_robot, camMatrix_, distCoeffs_,
  //                 rVectorRobot, tVectorRobot);
  //  } catch (cv::Exception &exp) {
  //    RCLCPP_INFO(this->get_logger(), "Not 2 solvePnP!");
  //  }

  //  cv::Mat c_R_r;
  //  cv::Rodrigues(rVectorRobot, c_R_r);
  //  cv::Vec3d c_P_cr = tVectorRobot;
  //  cv::Mat c_T_r =
  //      (cv::Mat_<double>(4, 4) << c_R_r.at<double>(0, 0),
  //       c_R_r.at<double>(0, 1), c_R_r.at<double>(0, 2), c_P_cr[0],
  //       c_R_r.at<double>(1, 0), c_R_r.at<double>(1, 1), c_R_r.at<double>(1, 2),
  //       c_P_cr[1], c_R_r.at<double>(2, 0), c_R_r.at<double>(2, 1),
  //       c_R_r.at<double>(2, 2), c_P_cr[2], 0, 0, 0, 1);

  //  cv::Mat w_T_r = w_T_c * c_T_r;

  //  geometry_msgs::msg::TransformStamped t;

  //  t.header.stamp = this->get_clock()->now();
  //  t.header.frame_id = world_name_.c_str();
  //  t.child_frame_id = robot_name_.c_str();

  //  t.transform.translation.x = w_T_r.at<double>(0, 3) / 100;
  //  t.transform.translation.y = w_T_r.at<double>(1, 3) / 100;
  //  t.transform.translation.z = w_T_r.at<double>(2, 3) / 100;

  //  cv::Mat w_R_r =
  //      (cv::Mat_<double>(3, 3) << w_T_r.at<double>(0, 0),
  //       w_T_r.at<double>(0, 1), w_T_r.at<double>(0, 2), w_T_r.at<double>(1, 0),
  //       w_T_r.at<double>(1, 1), w_T_r.at<double>(1, 2), w_T_r.at<double>(2, 0),
  //       w_T_r.at<double>(2, 1), w_T_r.at<double>(2, 2));
  //  cv::Mat r;
  //  cv::Rodrigues(w_R_r, r);
  //  tf2::Quaternion q;
  //  q.setRPY(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2));
  //  t.transform.rotation.x = q.x();
  //  t.transform.rotation.y = q.y();
  //  t.transform.rotation.z = q.z();
  //  t.transform.rotation.w = q.w();

  //  tf_robot_->sendTransform(t);
  //}

  // estimation blocks position
  for (int i = 0; i < count_block_; ++i) {
    auto point_block = point_blocks[i];
    auto pixel_block = pixel_blocks[i];
    cv::Mat rVectorBlock;
    cv::Mat tVectorBlock;
    try {
      cv::solvePnP(point_block, pixel_block, camMatrix_, distCoeffs_,
                   rVectorBlock, tVectorBlock);
    } catch (cv::Exception &exp) {
      RCLCPP_INFO(this->get_logger(), "Not 3 solvePnP!");
    }

    cv::Mat c_R_b;
    cv::Rodrigues(rVectorBlock, c_R_b);
    cv::Vec3d c_P_cb = tVectorBlock;
    cv::Mat c_T_b =
        (cv::Mat_<double>(4, 4) << c_R_b.at<double>(0, 0),
         c_R_b.at<double>(0, 1), c_R_b.at<double>(0, 2), c_P_cb[0],
         c_R_b.at<double>(1, 0), c_R_b.at<double>(1, 1), c_R_b.at<double>(1, 2),
         c_P_cb[1], c_R_b.at<double>(2, 0), c_R_b.at<double>(2, 1),
         c_R_b.at<double>(2, 2), c_P_cb[2], 0, 0, 0, 1);

    cv::Mat w_T_b = w_T_c * c_T_b;

    geometry_msgs::msg::TransformStamped t;
    pose_interfaces::msg::Pose pose;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = world_name_.c_str();
    t.child_frame_id = ("block_"+std::to_string(i)).c_str();

    t.transform.translation.x = w_T_b.at<double>(0, 3) / 100;
    t.transform.translation.y = w_T_b.at<double>(1, 3) / 100;
    t.transform.translation.z = w_T_b.at<double>(2, 3) / 100;
    pose.position.x = w_T_b.at<double>(0, 3) / 100;
    pose.position.y = w_T_b.at<double>(1, 3) / 100;
    pose.position.z = w_T_b.at<double>(2, 3) / 100;

    cv::Mat w_R_b =
        (cv::Mat_<double>(3, 3) << w_T_b.at<double>(0, 0),
         w_T_b.at<double>(0, 1), w_T_b.at<double>(0, 2), w_T_b.at<double>(1, 0),
         w_T_b.at<double>(1, 1), w_T_b.at<double>(1, 2), w_T_b.at<double>(2, 0),
         w_T_b.at<double>(2, 1), w_T_b.at<double>(2, 2));
    cv::Mat r;
    cv::Rodrigues(w_R_b, r);
    pose.orientation.x = r.at<double>(0, 0) * 180 / 3.14;
    pose.orientation.y = r.at<double>(0, 1) * 180 / 3.14;
    pose.orientation.z = r.at<double>(0, 2) * 180 / 3.14;
    block_coord_pub_[i]->publish(pose);
    tf2::Quaternion q;
    q.setRPY(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_block_[i]->sendTransform(t);
    //if (i == 0) {
    //  pub_->publish(t);
    //}
  }

}
