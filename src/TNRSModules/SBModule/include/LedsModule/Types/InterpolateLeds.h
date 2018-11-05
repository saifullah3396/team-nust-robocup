/**
 * @file SBModule/include/LedsModule/Types/InterpolateLeds.h
 *
 * This file declares the class InterpolateLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "SBModule/include/LedsModule/LedsModule.h"

/**
 * @class InterpolateLeds
 * @brief A class for interpolating leds to required intensities
 */
class InterpolateLeds : public LedsModule
{
public:
  /**
   * Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   */
  InterpolateLeds(SBModule* sbModule, const BehaviorConfigPtr& config) :
    LedsModule(sbModule, config, "InterpolateLeds"),
    execTime(0.f)
  {
  }

  /**
   * Destructor
   */
  ~InterpolateLeds()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void update();
  void finish();
  
private:
  //! Initial led intensities
  vector<float> ledsI;

  //! Difference with desired light intensity
  vector<float> ledsDelta;

  //! Time for leds interpolation
  float execTime;
};

typedef boost::shared_ptr<InterpolateLeds> InterpolateLedsPtr;
