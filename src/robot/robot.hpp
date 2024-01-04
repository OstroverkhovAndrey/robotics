
#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include "base/math/matrix.hpp"

class Robot {
public:
    virtual bool move(Matrix u) = 0;
    virtual Matrix get_sensor_measurement() = 0;
    virtual Matrix get_real_state() = 0;

    virtual ~Robot() {}

private:

};

#endif // _ROBOT_HPP_
