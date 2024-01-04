
#ifndef _KALMAN_FILTER_HPP_
#define _KALMAN_FILTER_HPP_

#include <vector>

#include "base/math/matrix.hpp"
#include "base/random_variable/normal_distribution.hpp"

class KalmanFilter {
public:
    KalmanFilter();
    // получает управление переданное роботу и изменяет оценку состояния робота
    // на основе этого управления
    bool move(Matrix u);

    // получает измерение датчика робота и на основе этого измерения 
    // уточняет состояние робота
    bool add_measurement(Matrix sensor_measurement);

    std::vector<NormalDistribution> move_and_measurement(Matrix u, Matrix sensor_measurement);

    // возвращает нормальное распределение, положение робота в заданный моммент
    NormalDistribution get_estimate_robot_state();

private:
    Matrix F;
    Matrix B;
    Matrix Q;
    Matrix H;
    Matrix R;

    NormalDistribution estimate_robot_state;

};

#endif //_KALMAN_FILTER_HPP_

