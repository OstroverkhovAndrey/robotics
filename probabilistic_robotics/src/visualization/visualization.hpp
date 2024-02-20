
#ifndef _VISUALIZATION_HPP_
#define _VISUALIZATION_HPP_

#include "base/random_variable/normal_distribution.hpp"

class Visualization {
public:
    //Visualization();
    virtual bool draw() = 0;
    virtual bool drawNormalDistribution(NormalDistribution& nd, int color) = 0;
    virtual bool drawRandomPoint(const Matrix& point) = 0;
    
    virtual ~Visualization() {}

private:
    

};

#endif // _VISUALIZATION_HPP_

