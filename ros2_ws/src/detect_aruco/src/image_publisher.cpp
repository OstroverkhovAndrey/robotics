
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher"), inputVideo(), count_(0) {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "image_with_undetect_markers", 10);
        timer_ = this->create_wall_timer(1000ms, 
                std::bind(&ImagePublisher::timer_callback, this));

        imageFlag = true;
        if (imageFlag) {
            inputVideo.open("./src/detect_aruco/resources/img_%02d.png");
            inputVideo.grab();
            inputVideo.retrieve(img_);
        } else {
            inputVideo.open(0);
            inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
            inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
            inputVideo.set(cv::CAP_PROP_FPS, 10);
            inputVideo.set(cv::CAP_PROP_FOURCC,
                    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            inputVideo.set(cv::CAP_PROP_BUFFERSIZE, 1);
        }
    }

private:
    void timer_callback() {
        cv::Mat image;
        if (imageFlag) {
            image = img_;
        } else {
            inputVideo.grab();
            inputVideo.retrieve(image);
        }
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
            .toImageMsg();
        publisher_->publish(*msg_.get());

        RCLCPP_INFO(this->get_logger(), "Image %i published", count_++);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture inputVideo;
    cv::Mat img_;
    bool imageFlag;
    int count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
