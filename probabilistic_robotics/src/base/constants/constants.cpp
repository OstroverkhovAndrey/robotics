
#include "base/constants/constants.hpp"
#include "base/math/matrix.hpp"

#include <iostream>
#include <vector>

bool Constants::create_consts_flag = false;
std::map<std::string, std::any> Constants::consts = std::map<std::string, std::any>();



std::map<std::string, std::any>& Constants::GetConstants(std::string consts_name) {
    if (!Constants::create_consts_flag) {
        Constants c(consts_name);
        Constants::create_consts_flag = true;
    }
    return Constants::consts;
}

std::map<std::string, std::any>& Constants::GetConstants() {
    return Constants::consts;
}

Constants::Constants(std::string consts_name) {
    if (consts_name == "simple_kalman_filter") {
        std::cout << consts_name << std::endl;
        Constants::consts["dimension"] = 2;
        Constants::consts["d_t"] = 0.1f;
        Constants::consts["x_0"] = Matrix(std::vector<std::vector<float>>({{100}, {100}}));
        Constants::consts["P_0"] = Matrix({{40.0, 20.0}, {20.0, 40.0}});
        Constants::consts["F"] = Matrix({{1.0, std::any_cast<float>(Constants::consts["d_t"])}, {0.0, 1.0}});
        Constants::consts["B"] = Matrix(std::vector<std::vector<float>>(
                    {{std::any_cast<float>(Constants::consts["d_t"])*
                    std::any_cast<float>(Constants::consts["d_t"])/2},
                    {std::any_cast<float>(Constants::consts["d_t"])}}));
        Constants::consts["Q"] = Matrix({{100.0, 0.0}, {0.0, 150.0}});
        Constants::consts["H"] = Matrix({{1.0, 0.0}, {0.0, 1.0}});
        Constants::consts["R"] = Matrix({{120.0, -50.0}, {-50.0, 70.0}});
    }        
}
