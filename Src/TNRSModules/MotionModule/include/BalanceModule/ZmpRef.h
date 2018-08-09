/**
 * @file MotionModule/include/KinematicsModule/ZmpRef.h
 *
 * This file defines the struct ZmpRef
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "Utils/include/MathsUtils.h"

/**
 * @struct ZmpRef
 * @brief Desired zmp x-y references in future
 */
struct ZmpRef
{
  /**
   * Constructor
   */ 
  ZmpRef() : length(0) { xy.resize(2); } // 2 for x-y dimensions
  
  bool setLength(const unsigned& length) {
    if (length > 0) {
      xy[0].resize(length);
      xy[1].resize(length);
      this->length = length;
    } else {
      return false;
    }
  }
  
  /**
   * Validates the sizes of x and y references sets them.
   * @param x Reference targets in X
   * @param x Reference targets in Y
   * 
   * @return true if the references are set successfully
   */ 
  bool setRef(const VectorXf& x, const VectorXf& y) {
    if (x.size() == y.size())
      length = x.size(); 
    else 
      return false;
    if (length > 0) {
      this->xy[0] = x;
      this->xy[1] = y;
    } else {
      return false;
    }
  }
  
  //! xy referencs of size length
  vector<VectorXf> xy;
  
  //! Number of future references
  size_t length;
};
