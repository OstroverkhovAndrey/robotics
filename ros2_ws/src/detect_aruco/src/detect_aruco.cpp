
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "detect_aruco/msg/point_in_image.hpp"
#include "detect_aruco/msg/marker_in_image.hpp"
#include "detect_aruco/msg/markers_in_image.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

class DetectAruco : public rclcpp::Node {
public:
    DetectAruco() : Node("detect_aruco") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "image_with_undetect_markers", 10, std::bind(
                &DetectAruco::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "image_with_detect_markers", 10);
        markers_publisher_ = this->create_publisher<detect_aruco::msg::MarkersInImage>(
                "coordinates_markers_in_image", 10);
    }
private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const {
	RCLCPP_INFO(this->get_logger(), "I get image!");

        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat imageCopy;
        image.copyTo(imageCopy);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

	    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        //std::cout << "markers ids\n";
        //for (auto & id : ids) {
        //    std::cout << id << " ";
        //}std::cout << "\n\n";

        sensor_msgs::msg::Image::SharedPtr msg_pub =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imageCopy).toImageMsg();
        publisher_->publish(*msg_pub.get());

        std::vector<detect_aruco::msg::MarkerInImage> markers;
        for (size_t i = 0; i < ids.size(); ++i) {
            detect_aruco::msg::MarkerInImage marker;
            marker.marker_id = ids[i];
            marker.up_left.x = corners[i][0].x;
            marker.up_left.y = corners[i][0].y;
            marker.up_right.x = corners[i][1].x;
            marker.up_right.y = corners[i][1].y;
            marker.down_right.x = corners[i][2].x;
            marker.down_right.y = corners[i][2].y;
            marker.down_left.x = corners[i][3].x;
            marker.down_left.y = corners[i][3].y;
            markers.push_back(marker);
        }

        detect_aruco::msg::MarkersInImage coordinates_msg;
        coordinates_msg.count_markers = ids.size();
        coordinates_msg.markers = markers;
        markers_publisher_->publish(coordinates_msg);

    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<detect_aruco::msg::MarkersInImage>::SharedPtr markers_publisher_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectAruco>());
    rclcpp::shutdown();
    return 0;
}
