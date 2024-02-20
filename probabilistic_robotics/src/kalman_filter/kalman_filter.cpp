
#include "kalman_filter/kalman_filter.hpp"
#include "base/constants/constants.hpp"

#include <iostream>

KalmanFilter::KalmanFilter() : estimate_robot_state(0) {
    auto c = Constants::GetConstants();
    F = std::any_cast<Matrix>(c["F"]);
    B = std::any_cast<Matrix>(c["B"]);
    Q = std::any_cast<Matrix>(c["Q"]);
    H = std::any_cast<Matrix>(c["H"]);
    R = std::any_cast<Matrix>(c["R"]);
    NormalDistribution nd(std::any_cast<int>(c["dimension"]));
    nd.SetMean(std::any_cast<Matrix>(c["x_0"]));
    nd.SetCovariance(std::any_cast<Matrix>(c["P_0"]));
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
