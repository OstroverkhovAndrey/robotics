
#ifndef _CONSTANTS_HPP_
#define _CONSTANTS_HPP_

#include <vector>

#include "base/math/matrix.hpp"

class Constants {
public:
    float d_t = 0.1;

    Matrix x_0 = Matrix(std::vector<std::vector<float>>({{100}, {100}}));

    Matrix P_0 = Matrix({{40.0, 20.0}, {20.0, 40.0}});

    Matrix F = Matrix({{1.0, d_t}, {0.0, 1.0}});

    Matrix B = Matrix(std::vector<std::vector<float>>({{d_t*d_t/2}, {d_t}}));

    Matrix Q = Matrix({{100.0, 0.0}, {0.0, 150.0}});

    Matrix H = Matrix({{1.0, 0.0}, {0.0, 1.0}});

    Matrix R = Matrix({{120.0, -50.0}, {-50.0, 70.0}});


};

#endif // _CONSTANTS_HPP_
