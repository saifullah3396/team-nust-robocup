/**
 * @file SBModule/src/LedsModule/LedsModule.cpp
 *
 * This file implements the class LedsModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "SBModule/include/LedsModule/LedsModule.h"
#include "SBModule/include/LedsModule/Types/DirectLeds.h"
#include "SBModule/include/LedsModule/Types/InterpolateLeds.h"

boost::shared_ptr<LedsModule> LedsModule::getType(
  SBModule* sbModule, const BehaviorConfigPtr& cfg) 
{ 
  LedsModule* lm;
  switch (cfg->type) {
      case (unsigned) SBLedsTypes::DIRECT_LEDS: 
        lm = new DirectLeds(sbModule, cfg); break;
      case (unsigned) SBLedsTypes::INTERPOLATE_LEDS: 
        lm = new InterpolateLeds(sbModule, cfg); break;
      default: lm = new DirectLeds(sbModule, cfg); break;
  }
  return boost::shared_ptr<LedsModule>(lm);
}

SBLedsConfigPtr LedsModule::getBehaviorCast()
{
  return boost::static_pointer_cast <SBLedsConfig> (config);
}
