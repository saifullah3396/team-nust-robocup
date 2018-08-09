/**
 * @file MotionModule/TrajectoryPlanner/TrajOptimizer.h
 *
 * This file declares the class TrajOptimizer
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Jul 2018  
 */
 
#pragma once
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/NLOptimizer.h"

/**
 * @class TrajOptimizer
 * @brief The base class for optimization of joint trajectories
 */
class TrajOptimizer : public NLOptimizer
{
public:
  /**
   * Constructor for this class with given knots.
   */
  TrajOptimizer(
    MotionModule* motionModule,
    const unsigned& chainIndex,
    const unsigned& baseLeg) :
    kM(motionModule->getKinematicsModule()), 
    chainIndex(chainIndex),
    baseLeg(baseLeg)
  {
    stepSize = kM->getCycleTime();
    chainSize = kM->getChainSize(chainIndex);
    velLimits = kM->getChainVelLimits(chainIndex);
    for (int i = 0; i < velLimits.size(); ++i)
      velLimits[i] -= 0.15;
  }
  
  /**
   * Default destructor for this class.
   */
  virtual ~TrajOptimizer()
  {
  }
  
protected:
  //! Taken as either left leg or right leg. Used for zmp constraints
  //! The base leg is considered as the inertial frame of reference for 
  //! whole body motion.
  unsigned baseLeg; 
  unsigned chainIndex;
  unsigned chainSize;
  float stepSize;
  //! Velocity bounds for the current chain.
  RowVectorXf velLimits;
  
  boost::shared_ptr<KinematicsModule> kM;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
