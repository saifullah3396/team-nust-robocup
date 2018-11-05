/**
 * @file SBModule/src/WhistleDetector/WhistleDetector.cpp
 *
 * This file implements the class WhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "SBModule/include/WhistleDetector/WhistleDetector.h"
#include "SBModule/include/WhistleDetector/Types/AKWhistleDetector.h"

boost::shared_ptr<WhistleDetector> WhistleDetector::getType(
  SBModule* sbModule, const BehaviorConfigPtr& cfg) 
{ 
  WhistleDetector* detector;
  switch (cfg->type) {
		case (unsigned) SBWDTypes::AK_WHISTLE_DETECTOR: 
			detector = new AKWhistleDetector(sbModule, cfg); break;
		default: detector = new AKWhistleDetector(sbModule, cfg); break;
  }
  return WhistleDetectorPtr(detector);
}

SBWDConfigPtr WhistleDetector::getBehaviorCast()
{
  return boost::static_pointer_cast <SBWDConfig> (config);
}

void WhistleDetector::whistleAction()
{
  OVAR(bool, SBModule::whistleDetected) = true;
}
