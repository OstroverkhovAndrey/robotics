
#include <iostream>
#include <unistd.h>
#include <vector>

#include "visualization/visualization_opencv.hpp"
#include "base/random_variable/normal_distribution.hpp"
#include "base/math/matrix.hpp"
#include "base/constants/constants.hpp"

#include "robot/robot.hpp"
#include "robot/robot_emulation.hpp"

#include "base/regulator/regulator.hpp"

#include "kalman_filter/kalman_filter.hpp"

int main (int argc, char *argv[]) {
    int i = 0;
    auto c = Constants::GetConstants(argv[1]);
    std::shared_ptr<Robot> robot = std::make_shared<RobotEmulation>();
    std::shared_ptr<Regulator> reg = std::make_shared<Regulator>();

    std::shared_ptr<KalmanFilter> filter = std::make_shared<KalmanFilter>();
    //NormalDistribution nd = filter->get_estimate_robot_state();

    KalmanFilterVisualization visualization(1280, 720);
    visualization.drawRandomPoint(robot->get_real_state());

    for (int i = 0; i < 9; ++i)
    {
        Matrix u = reg->get_u(i*std::any_cast<float>(c["d_t"]));
        robot->move(u);
        Matrix sensor_m = robot->get_sensor_measurement();
        std::vector<NormalDistribution> v_nd = filter->move_and_measurement(u, sensor_m);
        visualization.drawNormalDistribution(v_nd[0], 1);
        visualization.drawNormalDistribution(v_nd[1], 2);
        visualization.drawNormalDistribution(v_nd[2], 0);
        visualization.drawNormalDistribution(v_nd[3], 1);
        visualization.drawRandomPoint(robot->get_real_state());
    }

    //visualization.drawRandomPoint(robot->get_sensor_measurement());
    //visualization.drawNoramlDistribution(nd, 1);

    while (true) {
        std::cout << "Current iteration: " << i++ << std::endl;
        //robot->move(reg->get_u(i*c.d_t));
        //Matrix robot_state = robot->get_sensor_measurement();
        //std::cout << "robot_state: " << robot_state << std::endl;
        //visualization.drawRandomPoint(robot->get_sensor_measurement());
        visualization.draw();
        switch(visualization.key()) {
            case 'e':
                return 0;
        }
        sleep(1);
    }
    return 0;
}
