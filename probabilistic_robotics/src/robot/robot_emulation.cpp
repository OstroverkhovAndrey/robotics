
#include <iostream>
#include <any>

#include "robot/robot_emulation.hpp"
#include "base/constants/constants.hpp"
#include "base/random_variable/normal_distribution.hpp"

RobotEmulation::RobotEmulation() {
    auto c = Constants::GetConstants();
    robot_state_ = std::any_cast<Matrix>(c["x_0"]);
    F = std::any_cast<Matrix>(c["F"]);
    B = std::any_cast<Matrix>(c["B"]);
    Q = std::any_cast<Matrix>(c["Q"]);
    R = std::any_cast<Matrix>(c["R"]);
    H = std::any_cast<Matrix>(c["H"]);
}

bool RobotEmulation::move(Matrix u) {
    Matrix new_robot_state;
    new_robot_state = F * robot_state_ + B * u;
    NormalDistribution nd(robot_state_.N());
    nd.SetMean(new_robot_state);
    nd.SetCovariance(Q);
    new_robot_state = nd.GetRandomPoint();
    robot_state_ = new_robot_state;
    return true;
}

Matrix RobotEmulation::get_sensor_measurement() {
    NormalDistribution nd(robot_state_.N());
    nd.SetMean(robot_state_);
    nd.SetCovariance(R);
    Matrix ans = nd.GetRandomPoint();
    ans = H * ans;
    return ans;
}

