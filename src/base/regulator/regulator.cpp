
#include <cmath>
#include <vector>

#include "base/regulator/regulator.hpp"

Matrix Regulator::get_u(float t) {
    //return Matrix(std::vector<std::vector<float>>({{(float)std::sin(t)}}));
    return Matrix(std::vector<std::vector<float>>({{(float)1000}}));
}
