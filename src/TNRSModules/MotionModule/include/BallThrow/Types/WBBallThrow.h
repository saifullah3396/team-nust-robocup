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
template <typename Scalar>
class WBBallThrow : public BallThrow<Scalar>
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
    BallThrow<Scalar>(motionModule, config, "WBBallThrow")
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
private:

  void executeArmsTrajs(
    const vector<vector<Scalar> >& jointTrajectories, 
    const Scalar& stepTime);
  
  void grabBallAction();
  void retractAction();
  void throwBallAction();
};

typedef boost::shared_ptr<WBBallThrow<MType> > WBBallThrowPtr;
