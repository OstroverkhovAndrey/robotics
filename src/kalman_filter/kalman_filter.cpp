
#include "kalman_filter/kalman_filter.hpp"
#include "base/constants/constants.hpp"

#include <iostream>

KalmanFilter::KalmanFilter() : estimate_robot_state(0) {
    Constants c;
    F = c.F;
    B = c.B;
    Q = c.Q;
    H = c.H;
    R = c.R;
    NormalDistribution nd(2);
    nd.SetMean(c.x_0);
    nd.SetCovariance(c.P_0);
    estimate_robot_state = nd;
}

NormalDistribution KalmanFilter::get_estimate_robot_state() {
    return estimate_robot_state;
}

std::vector<NormalDistribution> KalmanFilter::move_and_measurement(Matrix u,
        Matrix z) {
    std::vector<NormalDistribution> ans;
    // предыдущее состояние
    ans.push_back(estimate_robot_state);
    
    // новое состояние после движения
    Matrix x = F * estimate_robot_state.GetMean() + B * u;
    Matrix P = F * estimate_robot_state.GetCovariance() + F.T() + Q;
    NormalDistribution nd_1(2);
    nd_1.SetMean(x);
    nd_1.SetCovariance(P);
    ans.push_back(nd_1);

    // нормальное распределение измерения от датчика
    NormalDistribution nd_2(2);
    nd_2.SetMean(z);
    nd_2.SetCovariance(R);
    ans.push_back(nd_2);

    // новая оценка состояния робота
    NormalDistribution nd_3(2);
    Matrix K = P * H.T() * (H * P + H.T() + R).inverse();
    Matrix new_x = x + K * (z - H * x);
    Matrix new_P = P - K * H * P;
    nd_3.SetMean(new_x);
    nd_3.SetCovariance(new_P);
    ans.push_back(nd_3);
    estimate_robot_state = nd_3;

    return ans;
}
