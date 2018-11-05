/**
 * @file MotionModule/include/KinematicsModule/LinkInfo.h
 *
 * This file defines the struct LinkInfo
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/make_shared.hpp>
#include "Utils/include/MathsUtils.h"
#include "LinkChain.h"

/**
 * @struct LinkInfo
 * @brief A struct that holds information about the link properties
 */
template<typename Scalar>
struct LinkInfo
{
  /**
   * Constructor
   */ 
  LinkInfo() 
  {
  }
  
  //! Link mass
  Scalar mass;
  
  //! Partial mass with respect to totalChains mass
  Scalar partialMass;
  
  //! Link inertia tensor matrix
  Matrix<Scalar, 3, 3> inertia;
  
  //! Link inertia transformation matrix
  Matrix<Scalar, 3, 3> inertiaTrans;
  
  //! Link center of mass vector
  Matrix<Scalar, 4, 1> com;
  
  //! Associated link chain
  boost::shared_ptr<LinkChain<Scalar> > chain;
};
template struct LinkInfo<float>;
template struct LinkInfo<double>;
