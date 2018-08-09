/**
 * @file MotionModule/include/PostureModule/Types/InterpToPosture.h
 *
 * This file declares the class InterpToPosture
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "MotionModule/include/PostureModule/PostureModule.h"

/**
 * @class InterpToPosture
 * @brief A class that interpolates the joints from initial state to
 *   final state based on quintic splines with zero initial and final
 *   velocities and accelerations
 */ 
class InterpToPosture : public PostureModule
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  InterpToPosture(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config) :
    PostureModule(motionModule, config, "InterpToPosture")
  {
  }

  /**
   * Default destructor for this class.
   */
  ~InterpToPosture()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
};

typedef boost::shared_ptr<InterpToPosture> InterpToPosturePtr;
