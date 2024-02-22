
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

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
    }
private:
    void topic_callback(const sensor_msgs::msg::Image & msg) const {
	RCLCPP_INFO(this->get_logger(), "I get image!");

        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        std::cout << "markers ids\n";
        for (auto & id : ids) {
            std::cout << id << " ";
        }std::cout << "\n\n";
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectAruco>());
    rclcpp::shutdown();
    return 0;
}
