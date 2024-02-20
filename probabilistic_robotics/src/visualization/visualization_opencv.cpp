
#include <iostream>

#include "visualization/visualization_opencv.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

KalmanFilterVisualization::KalmanFilterVisualization(int size_image_x,
        int size_image_y) :
    size_image_x_(size_image_x),
    size_image_y_(size_image_y),
    image_(size_image_y_, size_image_x_, CV_8UC3) {
        drawBackground();
}

KalmanFilterVisualization::~KalmanFilterVisualization(){}

void KalmanFilterVisualization::drawRect() {
    for (int i = 10; i < 110; ++i) {
        for (int j = 10; j < 110; ++j) {
            drawPixel(i, j, cv::Vec3b(200, 200, 200));
            //cv::Vec3b & pixel = image_.at<cv::Vec3b>(j, i);
            //cv::Vec3b color = cv::Vec3b(100, 100, 100);
            //pixel = color;
        }
    }
}

bool KalmanFilterVisualization::drawPixel(int x, int y, cv::Vec3b color) {
    y = -y+size_image_y_-1;
    if (0 <= x && x < size_image_x_ && 0 <= y && y < size_image_y_) {
        image_.at<cv::Vec3b>(y, x) = color;
    }
    return true;
}

cv::Vec3b KalmanFilterVisualization::getPixel(int x, int y) {
    y = -y+size_image_y_-1;
    if (0 <= x && x < size_image_x_ && 0 <= y && y < size_image_y_) {
        return image_.at<cv::Vec3b>(y, x);
    }
    return cv::Vec3b(0, 0, 0);
}

bool KalmanFilterVisualization::drawBackground () {
    for (int i = 0; i < size_image_x_; ++i) {
        for (int j = 0; j < size_image_y_; ++j) {
            drawPixel(i, j, cv::Vec3b(0, 0, 0));
        }
    }
    //drawRect();
    return true;
}

bool KalmanFilterVisualization::draw () {
    imshow("out", image_);
    return true;
}

bool KalmanFilterVisualization::drawNormalDistribution(NormalDistribution& distribution, int color_num) {
    if (color_num < 0 || color_num > 2) {
        color_num = 2;
    }
    int x = (int)distribution.GetMean().GetElement(0, 0);
    int y = (int)distribution.GetMean().GetElement(1, 0);

    int k = 500;
    for (int i = x-k; i < x+k; ++i) {
        for (int j = y-k; j < y+k; ++j) {
            Matrix point(2, 1);
            point.SetElement(0, 0, i);
            point.SetElement(1, 0, j);
            cv::Vec3b color = getPixel(i, j);
            float color_f = distribution.GetPoint(point)*10000000;
            color_f += color[color_num];
            if (color_f > 255) {
                color[color_num] = (char)255;
            } else if (color_f < 0) {
                color[color_num] = (char)0;
            } else {
                color[color_num] = char(color_f);
            }
            drawPixel(i, j, color);
        }
    }
    return true;
}

char KalmanFilterVisualization::key() {
    return (char)cv::pollKey();
}

bool KalmanFilterVisualization::drawRandomPoint(const Matrix& point) {
    int x = point.GetElement(0, 0), y = point.GetElement(1, 0);
    int d = 5;
    for (int i = x-d; i < x+d; ++i) {
        for (int j = y-d; j < y+d; ++j) {
            drawPixel(i, j, cv::Vec3b(25, 25, 25));
        }
    }
    return true;
}
