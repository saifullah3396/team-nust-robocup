/**
 * @file SBModule/include/LedsModule/Types/DirectLeds.h
 *
 * This file declares the class DirectLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "SBModule/include/LedsModule/LedsModule.h"

/**
 * @class DirectLeds
 * @brief A class for directly sending leds to required intensities
 */
class DirectLeds : public LedsModule
{
public:
  /**
   * Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   */
  DirectLeds(SBModule* sbModule, const BehaviorConfigPtr& config) :
    LedsModule(sbModule, config, "DirectLeds")
  {
  }

  /**
   * Destructor
   */
  ~DirectLeds()
  {
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate() ;
  void update();
  void finish();
};

typedef boost::shared_ptr<DirectLeds> DirectLedsPtr;
