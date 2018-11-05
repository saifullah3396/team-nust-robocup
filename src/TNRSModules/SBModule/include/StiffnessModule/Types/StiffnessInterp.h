/**
 * @file SBModule/include/StiffnessModule/Types/StiffnessInterp.h
 *
 * This file declares the class StiffnessInterp
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#pragma once

#include "SBModule/include/StiffnessModule/StiffnessModule.h"

class StiffnessInterp : public StiffnessModule
{
public:
  /**
   * Constructor
   *
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   */
  StiffnessInterp(
	SBModule* sBModule,
	const BehaviorConfigPtr& config) :
    StiffnessModule(sBModule, config, "StiffnessToInterp"),
    execTime(0.f)
  {
  }

  /**
   * Destructor
   */
  ~StiffnessInterp()
  {
  }
  
  /**
   * Derived from Behavior
   */
  void initiate() ;
  void update();
  void finish();
  
private:
  float execTime;
};

typedef boost::shared_ptr<StiffnessInterp> StiffnessInterpPtr;
