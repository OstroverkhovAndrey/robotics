
#ifndef _KALMAN_FILTER_VISUALIZATION_HPP_
#define _KALMAN_FILTER_VISUALIZATION_HPP_

#include <iostream>
#include <opencv2/highgui.hpp>

#include "base/math/matrix.hpp"
#include "base/random_variable/normal_distribution.hpp"

class KalmanFilterVisualization {
public:
    KalmanFilterVisualization(int size_image_x, int size_image_y);
    ~KalmanFilterVisualization();

    bool draw();
    char key();
    bool drawNoramlDistribution(NormalDistribution& distribution, int color_number);
    bool drawBackground();
    bool drawRandomPoint(Matrix point);

private:
    bool drawPixel(int x, int y, cv::Vec3b color);
    cv::Vec3b getPixel(int x, int y);
    void drawRect();

    int size_image_x_, size_image_y_;
    cv::Mat image_;
};

#endif // _KALMAN_FILTER_VISUALIZATION_HPP_
