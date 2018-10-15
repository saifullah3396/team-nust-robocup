/**
 * @file MotionModule/include/TrajectoryPlanner/TrajOptimizer.h
 *
 * This file declares the class TrajOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/NLOptimizer.h"

/**
 * @class TrajOptimizer
 * @brief The base class for optimization of joint trajectories
 */
template <typename Scalar>
class TrajOptimizer : public NLOptimizer
{
public:
  /**
   * Constructor for this class with given knots.
   */
  TrajOptimizer(
    MotionModule* motionModule,
    const unsigned& chainIndex,
    const unsigned& baseLeg);
  
  /**
   * Default destructor for this class.
   */
  virtual ~TrajOptimizer();
  
protected:
  //! Taken as either left leg or right leg. Used for zmp constraints
  //! The base leg is considered as the inertial frame of reference for 
  //! whole body motion.
  unsigned baseLeg; 
  unsigned chainIndex;
  unsigned chainSize;
  Scalar stepSize;
  //! Velocity bounds for the current chain.
  Matrix<Scalar, 1, Dynamic> velLimits;
  
  KinematicsModulePtr kM;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

