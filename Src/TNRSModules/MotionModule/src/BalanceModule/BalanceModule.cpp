/**
 * @file MotionModule/src/BalanceModule/BalanceModule.cpp
 *
 * This file implements the class BalanceModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 20 April 2017
 */

#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/BalanceModule/Types/MPComControl.h"
#include "MotionModule/include/BalanceModule/Types/PIDComControl.h"
#include "MotionModule/include/BalanceModule/Types/ZmpControl.h"
#include "ConfigManager/include/ConfigManager.h"

boost::shared_ptr<BalanceModule> BalanceModule::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  BalanceModule* bm;
  switch (cfg->type) {
      case (unsigned) MBBalanceTypes::MP_COM_CONTROL: 
        bm = new MPComControl(motionModule, cfg); break;
      case (unsigned) MBBalanceTypes::PID_COM_CONTROL: 
        bm = new PIDComControl(motionModule, cfg); break;
      case (unsigned) MBBalanceTypes::ZMP_CONTROL: 
        bm = new ZmpControl(motionModule, cfg); break;
      default: bm = new MPComControl(motionModule, cfg); break;
  }
  return boost::shared_ptr<BalanceModule>(bm);
}
