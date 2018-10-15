/**
 * @file MotionModule/include/MBManager.h
 *
 * This file declares the class MBManager.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#pragma once

#include "MotionModule/include/MTypeHeader.h"
#include "BehaviorManager/include/BehaviorManager.h"

//! Forward declaration
class MotionModule;

/**
 * @class MBManager 
 * @brief Handles the initiation and execution of motion behaviors
 */
template <typename Scalar>
class MBManager : public BehaviorManager
{
public:
  /**
   * Constructor
   * 
   * @param motionModule: Pointer to the base motion module
   */
  MBManager(MotionModule* motionModule);

  /**
   * Destructor
   */
  ~MBManager() {}
private:
  /**
   * Derived from BehaviorManager
   */
  bool
  makeBehavior(BehaviorPtr& behavior, const BehaviorConfigPtr& cfg);
  
  //! Pointer to the base motion behaviors module
  MotionModule* motionModule;
};

typedef boost::shared_ptr<MBManager<MType> > MBManagerPtr;
