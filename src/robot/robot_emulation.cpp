
#include <iostream>

#include "robot/robot_emulation.hpp"
#include "base/constants/constants.hpp"
#include "base/random_variable/normal_distribution.hpp"

RobotEmulation::RobotEmulation() {
    Constants c;
    robot_state_ = c.x_0;
    F = c.F;
    B = c.B;
    Q = c.Q;
    R = c.R;
    H = c.H;
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

