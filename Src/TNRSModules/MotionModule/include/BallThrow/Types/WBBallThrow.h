/**
 * @file MotionModule/BallThrow/Types/WBBallThrow.h
 *
 * This file declares the class WBBallThrow
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/BallThrow/BallThrow.h"

/**
 * @class WBBallThrow
 * @brief A class for defining a ball throw based on whole body motion
 *   of the robot
 */
class WBBallThrow : public BallThrow
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  WBBallThrow(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    BallThrow(motionModule, config, "WBBallThrow")
  {
  }

  /**
   * Default destructor for this class.
   */
  ~WBBallThrow()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
};

typedef boost::shared_ptr<WBBallThrow> WBBallThrowPtr;
