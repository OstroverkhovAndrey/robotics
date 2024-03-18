
#include <chrono>
#include <functional>
#include <string>

#include <unordered_map>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "rclcpp/rclcpp.hpp"
#include "detect_aruco/msg/point_in_image.hpp"
#include "detect_aruco/msg/marker_in_image.hpp"
#include "detect_aruco/msg/markers_in_image.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class EstimationRobotPosition : public rclcpp::Node {
public:
    EstimationRobotPosition() : Node("estimation_robot_position") {
        subscription_ = this->create_subscription<detect_aruco::msg::MarkersInImage>(
                "coordinates_markers_in_image", 10, std::bind(&EstimationRobotPosition::topic_callback, this, _1));

        robot_name_ =
            this->declare_parameter<std::string>("robot_name", "turtle");

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // [0] верхний левый угол
        // [1] верхний правый угол
        // [2] нижний правый угол
        // [3] нижний левый угол

        // координаты опорных меркеров в мировой системе координат
        countMarkersWorldCoordinateSystem_ = 4;

        markersWorldCoordinateSystem_[24] = std::vector<cv::Point3f>(4);
        markersWorldCoordinateSystem_[24][0] = cv::Point3f(0.0+1.0, 0.0+6.0, 0.0);
        markersWorldCoordinateSystem_[24][1] = cv::Point3f(0.0+6.0, 0.0+6.0, 0.0);
        markersWorldCoordinateSystem_[24][2] = cv::Point3f(0.0+6.0, 0.0+1.0, 0.0);
        markersWorldCoordinateSystem_[24][3] = cv::Point3f(0.0+1.0, 0.0+1.0, 0.0);

        markersWorldCoordinateSystem_[25] = std::vector<cv::Point3f>(4);
        markersWorldCoordinateSystem_[25][0] = cv::Point3f(0.0+1.0, 84.5-1.0, 0.0);
        markersWorldCoordinateSystem_[25][1] = cv::Point3f(0.0+6.0, 84.5-1.0, 0.0);
        markersWorldCoordinateSystem_[25][2] = cv::Point3f(0.0+6.0, 84.5-6.0, 0.0);
        markersWorldCoordinateSystem_[25][3] = cv::Point3f(0.0+1.0, 84.5-6.0, 0.0);

        markersWorldCoordinateSystem_[26] = std::vector<cv::Point3f>(4);
        markersWorldCoordinateSystem_[26][0] = cv::Point3f(54.5-6.0, 84.5-1.0, 0.0);
        markersWorldCoordinateSystem_[26][1] = cv::Point3f(54.5-1.0, 84.5-1.0, 0.0);
        markersWorldCoordinateSystem_[26][2] = cv::Point3f(54.5-1.0, 84.5-6.0, 0.0);
        markersWorldCoordinateSystem_[26][3] = cv::Point3f(54.5-6.0, 84.5-6.0, 0.0);

        markersWorldCoordinateSystem_[27] = std::vector<cv::Point3f>(4);
        markersWorldCoordinateSystem_[27][0] = cv::Point3f(54.5-6.0, 0.0+6.0, 0.0);
        markersWorldCoordinateSystem_[27][1] = cv::Point3f(54.5-1.0, 0.0+6.0, 0.0);
        markersWorldCoordinateSystem_[27][2] = cv::Point3f(54.5-1.0, 0.0+1.0, 0.0);
        markersWorldCoordinateSystem_[27][3] = cv::Point3f(54.5-6.0, 0.0+1.0, 0.0);

        // координаты маркеров робота в системе координат робота
        countMarkersRobotCoordinateSystem_ = 1;

        markersRobotCoordinateSystem_[23] = std::vector<cv::Point3f>(4);
        markersRobotCoordinateSystem_[23][0] = cv::Point3f(-6.0, -1.0, 0.0);
        markersRobotCoordinateSystem_[23][1] = cv::Point3f(-1.0, -1.0, 0.0);
        markersRobotCoordinateSystem_[23][2] = cv::Point3f(-1.0, -6.0, 0.0);
        markersRobotCoordinateSystem_[23][3] = cv::Point3f(-6.0, -6.0, 0.0);

        readCameraParameters("./src/detect_aruco/resources/cameraCalib.txt", camMatrix_, distCoeffs_);
    }

private:
    void topic_callback(const detect_aruco::msg::MarkersInImage & msg) {
        // координаты опорных точек в мировой ск и на изображенни
        std::vector<cv::Point3f> pointWorldCoordinateSystem;
        std::vector<cv::Point2f> pointWorldInPicture;
    
        // координаты точек робота в ск робота и на изображении
        std::vector<cv::Point3f> pointRobotCoordinateSystem;
        std::vector<cv::Point2f> pointRobotInPicture; 

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        for (int i = 0; i < (int)msg.markers.size(); ++i)
        {
            ids.push_back(msg.markers[i].marker_id);
            corners.push_back(std::vector<cv::Point2f>(4));
            corners[i][0].x = msg.markers[i].up_left.x;
            corners[i][0].y = msg.markers[i].up_left.y;
            corners[i][1].x = msg.markers[i].up_right.x;
            corners[i][1].y = msg.markers[i].up_right.y;
            corners[i][2].x = msg.markers[i].down_right.x;
            corners[i][2].y = msg.markers[i].down_right.y;
            corners[i][3].x = msg.markers[i].down_left.x;
            corners[i][3].y = msg.markers[i].down_left.y;
        }
    
        // заполняем векторы точек по найденным маркерам
        for (int i = 0; i < (int)ids.size(); ++i) {
            int id = ids[i];
            if (markersWorldCoordinateSystem_.find(id) != markersWorldCoordinateSystem_.end()) {
                pointWorldCoordinateSystem.push_back(markersWorldCoordinateSystem_[id][0]);
                pointWorldCoordinateSystem.push_back(markersWorldCoordinateSystem_[id][1]);
                pointWorldCoordinateSystem.push_back(markersWorldCoordinateSystem_[id][2]);
                pointWorldCoordinateSystem.push_back(markersWorldCoordinateSystem_[id][3]);
    
                pointWorldInPicture.push_back(corners[i][0]);
                pointWorldInPicture.push_back(corners[i][1]);
                pointWorldInPicture.push_back(corners[i][2]);
                pointWorldInPicture.push_back(corners[i][3]);
            }
            if (markersRobotCoordinateSystem_.find(id) != markersRobotCoordinateSystem_.end()) {
                pointRobotCoordinateSystem.push_back(markersRobotCoordinateSystem_[id][0]);
                pointRobotCoordinateSystem.push_back(markersRobotCoordinateSystem_[id][1]);
                pointRobotCoordinateSystem.push_back(markersRobotCoordinateSystem_[id][2]);
                pointRobotCoordinateSystem.push_back(markersRobotCoordinateSystem_[id][3]);
    
                pointRobotInPicture.push_back(corners[i][0]);
                pointRobotInPicture.push_back(corners[i][1]);
                pointRobotInPicture.push_back(corners[i][2]);
                pointRobotInPicture.push_back(corners[i][3]);
            }
        }
    
        cv::Mat rVectorWorldCoordinateSystem;
        cv::Mat tVectorWorldCoordinateSystem;


        std::cout << pointWorldInPicture.size() << std::endl;
        std::cout << pointRobotInPicture.size() << std::endl;
        std::cout << pointWorldCoordinateSystem.size() << std::endl;
        std::cout << pointRobotCoordinateSystem.size() << std::endl;
        std::cout << camMatrix_.size() << std::endl;
        std::cout << distCoeffs_.size() << std::endl;
        try {
            cv::solvePnP(pointWorldCoordinateSystem,
                    pointWorldInPicture,
                    camMatrix_, distCoeffs_,
                    rVectorWorldCoordinateSystem,
                    tVectorWorldCoordinateSystem);
        }
        catch (cv::Exception & exp) {
            RCLCPP_INFO(this->get_logger(), "Not 1 solvePnP!");
        }
    
        cv::Mat rVectorRobotCoordinateSystem;
        cv::Mat tVectorRobotCoordinateSystem;
        try {
            cv::solvePnP(pointRobotCoordinateSystem,
                    pointRobotInPicture,
                    camMatrix_, distCoeffs_,
                    rVectorRobotCoordinateSystem,
                    tVectorRobotCoordinateSystem);
        }
        catch (cv::Exception & exp) {
            RCLCPP_INFO(this->get_logger(), "Not 2 solvePnP!");
        }
    
        cv::Mat c_R_w;
        cv::Rodrigues(rVectorWorldCoordinateSystem, c_R_w);
        cv::Mat w_R_c = c_R_w.t();
        cv::Mat x = - w_R_c * tVectorWorldCoordinateSystem;
        cv::Vec3d w_P_wc = x;
        cv::Mat w_T_c = (cv::Mat_<double>(4, 4) << w_R_c.at<double>(0, 0),w_R_c.at<double>(0, 1),w_R_c.at<double>(0, 2),w_P_wc[0],
                                            w_R_c.at<double>(1, 0),w_R_c.at<double>(1, 1),w_R_c.at<double>(1, 2),w_P_wc[1],
                                            w_R_c.at<double>(2, 0),w_R_c.at<double>(2, 1),w_R_c.at<double>(2, 2),w_P_wc[2],
                                            0, 0, 0, 1);
    
        cv::Mat c_R_o;
        cv::Rodrigues(rVectorRobotCoordinateSystem, c_R_o);
        cv::Vec3d c_P_co = tVectorRobotCoordinateSystem;
        cv::Mat c_T_o = (cv::Mat_<double>(4, 4) << c_R_o.at<double>(0, 0),c_R_o.at<double>(0, 1),c_R_o.at<double>(0, 2),c_P_co[0],
                                            c_R_o.at<double>(1, 0),c_R_o.at<double>(1, 1),c_R_o.at<double>(1, 2),c_P_co[1],
                                            c_R_o.at<double>(2, 0),c_R_o.at<double>(2, 1),c_R_o.at<double>(2, 2),c_P_co[2],
                                            0, 0, 0, 1);
    
        cv::Mat w_T_o = w_T_c * c_T_o;
    
        //cout << "Mat: " << w_T_o << endl;
        cv::Mat c0 = (cv::Mat_<double>(4, 1) << 0.0, 0.0, 0.0, 1.0);
        cv::Mat c1 = (cv::Mat_<double>(4, 1) << 10.0, 0.0, 0.0, 1.0);
    
        //cout << "answer c0: " << w_T_o * c0 << endl;
        //cout << "answer c1: " << w_T_o * c1 << endl;
        
        c0 = w_T_o *c0;
        double x_0 = c0.at<double>(0);
        double y_0 = c0.at<double>(1);
    
        c1 = w_T_o *c1;
        double x_1 = c1.at<double>(0);
        double y_1 = c1.at<double>(1);
    
        //cout << "angle: " << calc_angle(x_1-x_0, y_1-y_0) * 180 / 3.14 << endl;
        //double ang = angle(x_0, x_1, y_0, y_1);
        double ang = angle(10, 0, x_1-x_0, y_1-y_0);
        //cout << "angle: " << ang << endl;
        RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, Angle: %f", x_0, y_0, ang);
    
        //return Vec3d(x_0, y_0, ang);

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        // w_T_o
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = robot_name_.c_str();

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = (w_T_o.at<double>(0, 3)+i_++)/100;
        t.transform.translation.y = w_T_o.at<double>(1, 3)/100;
        t.transform.translation.z = w_T_o.at<double>(2, 3)/100;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        std::cout << "!!!" << std::endl;
        cv::Mat w_R_o = (cv::Mat_<double>(3, 3) << w_T_o.at<double>(0, 0),w_T_o.at<double>(0, 1),w_T_o.at<double>(0, 2),
                                            w_T_o.at<double>(1, 0),w_T_o.at<double>(1, 1),w_T_o.at<double>(1, 2),
                                            w_T_o.at<double>(2, 0),w_T_o.at<double>(2, 1),w_T_o.at<double>(2, 2) );
        cv::Mat r;
        cv::Rodrigues(w_R_o, r);
        std::cout << "r: " << r << std::endl;
        tf2::Quaternion q;
        q.setRPY(r.at<double>(0, 0), r.at<double>(0, 1), r.at<double>(0, 2));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    double angle(double x1, double y1, double x2, double y2) {
        double t = (x1*x2+y1*y2)/(sqrt((double)x1*x1+y1*y1)*sqrt((double)x2*x2+y2*y2));
        if     (t<-1) t=-1;
        else if(t> 1) t= 1;
        double ans = acos(t);
        if (y2-y1 < 0) ans = 3.1415 * 2 - ans;
        return ans;
    }

    inline static bool readCameraParameters(std::string filename, cv::Mat & camMatrix, cv::Mat & distCoeffs) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        return true;
    }


    rclcpp::Subscription<detect_aruco::msg::MarkersInImage>::SharedPtr subscription_;
    int countMarkersWorldCoordinateSystem_;
    int countMarkersRobotCoordinateSystem_;
    std::unordered_map<int, std::vector<cv::Point3f>> markersWorldCoordinateSystem_;
    std::unordered_map<int, std::vector<cv::Point3f>> markersRobotCoordinateSystem_;
    cv::Mat camMatrix_, distCoeffs_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string robot_name_;
    int i_ = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EstimationRobotPosition>());
    rclcpp::shutdown();
    return 0;
}
