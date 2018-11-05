/**
 * @file SBModule/src/StiffnessModule/StiffnessModule.h
 *
 * This file implements the class StiffnessModule
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#include "SBModule/include/StiffnessModule/StiffnessModule.h"
#include "SBModule/include/StiffnessModule/Types/StiffnessInterp.h"

boost::shared_ptr<StiffnessModule> StiffnessModule::getType(
  SBModule* sbModule, 
	const BehaviorConfigPtr& cfg) 
{ 
  StiffnessModule* sm;
  switch (cfg->type) {
      case (unsigned) SBStiffnessTypes::STIFFNESS_INTERP: 
        sm = new StiffnessInterp(sbModule, cfg); break;
      default: sm = new StiffnessInterp(sbModule, cfg); break;
  }
  return StiffnessModulePtr(sm);
}

SBStiffnessConfigPtr StiffnessModule::getBehaviorCast()
{
  return 
		boost::static_pointer_cast <SBStiffnessConfig> (config);
}
