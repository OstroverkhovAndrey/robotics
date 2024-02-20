
#ifndef _KALMAN_FILTER_VISUALIZATION_HPP_
#define _KALMAN_FILTER_VISUALIZATION_HPP_

#include <iostream>
#include <opencv2/highgui.hpp>

#include "base/math/matrix.hpp"
#include "base/random_variable/normal_distribution.hpp"
#include "visualization/visualization.hpp"

class KalmanFilterVisualization : public Visualization {
public:
    KalmanFilterVisualization(int size_image_x, int size_image_y);
    ~KalmanFilterVisualization();

    bool draw() override;
    char key();
    bool drawNormalDistribution(NormalDistribution& distribution, int color_number) override;
    bool drawBackground();
    bool drawRandomPoint(const Matrix& point) override;

private:
    bool drawPixel(int x, int y, cv::Vec3b color);
    cv::Vec3b getPixel(int x, int y);
    void drawRect();

    int size_image_x_, size_image_y_;
    cv::Mat image_;
};

#endif // _KALMAN_FILTER_VISUALIZATION_HPP_
