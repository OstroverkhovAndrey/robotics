
#ifndef _REGULATOR_HPP_
#define _REGULATOR_HPP_

#include "base/math/matrix.hpp"

class Regulator {
public:
    Matrix get_u(float t);
};


#endif // _REGULATOR_HPP_

