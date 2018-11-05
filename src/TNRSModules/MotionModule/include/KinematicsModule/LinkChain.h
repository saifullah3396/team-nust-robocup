/**
 * @file MotionModule/include/KinematicsModule/LinkChain.h
 *
 * This file defines the struct LinkChain
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/make_shared.hpp>
#include "Utils/include/MathsUtils.h"

/**
 * @struct LinkChain
 * @brief A struct that holds information about a links chain
 */
template<typename Scalar>
struct LinkChain
{
  /**
   * Constructor
   */ 
  LinkChain(
    const unsigned& start = 0,
    const unsigned& size = 0,
    const Scalar& mass = Scalar(0),
    const Matrix<Scalar, 4, 4>& startT = Matrix<Scalar, 4, 4>::Identity(),
    const Matrix<Scalar, 4, 4>& endT = Matrix<Scalar, 4, 4>::Identity()) :
    start(start), size(size), mass(mass), startT(startT), endT(endT)
  {
  }
  
  //! LinkChain start index
  unsigned start;
  
  //! LinkChain size
  unsigned size;
  
  //! Total mass of the chain
  Scalar mass;
  
  //! Total masses of all the chains
  static Scalar totalChainsMass;
  
  //! LinkChain start transformation matrix
  Matrix<Scalar, 4, 4> startT;
  
  //! LinkChain end transformation matrix
  Matrix<Scalar, 4, 4> endT;
  
  //! LinkChain end effector definitions
  vector<Matrix<Scalar, 4, 4> > endEffectors;
};

template<typename Scalar> Scalar LinkChain<Scalar>::totalChainsMass = 0;

template struct LinkChain<float>;
template struct LinkChain<double>;
