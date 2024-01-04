
#ifndef _NORMAL_DISTRIBUTION_HPP_
#define _NORMAL_DISTRIBUTION_HPP_

#include <vector>

#include <ostream>

#include "base/math/matrix.hpp"

class NormalDistribution {
    friend std::ostream& operator << (std::ostream &os,
            const NormalDistribution & distribution);
public:
  NormalDistribution();
  NormalDistribution(int dimension);
  ~NormalDistribution();

  bool SetMean(std::vector<float> mean);
  bool SetCovariance(std::vector<std::vector<float>> covariance);
  

  bool SetMean(Matrix mean);
  bool SetCovariance(Matrix covariance);

  float GetPoint(Matrix x); // get value in point
  
  Matrix GetMean() {return mean_;}
  Matrix GetCovariance() {return covariance_;}

  Matrix GetRandomPoint();
  //bool python_close();

  NormalDistribution operator*(NormalDistribution & other);

  static bool python_flag;

private:
  int dimension_;
  Matrix mean_;
  Matrix covariance_;
};

#endif // _NORMAL_DISTRIBUTION_HPP_

