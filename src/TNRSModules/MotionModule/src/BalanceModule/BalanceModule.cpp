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
#include "Utils/include/ConfigManager.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"

template<typename Scalar>
boost::shared_ptr<BalanceModule<Scalar> > BalanceModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  BalanceModule<Scalar>* bm;
  switch (cfg->type) {
      case (unsigned) MBBalanceTypes::MP_COM_CONTROL: 
        bm = new MPComControl<Scalar>(motionModule, cfg); break;
      case (unsigned) MBBalanceTypes::PID_COM_CONTROL: 
        bm = new PIDComControl<Scalar>(motionModule, cfg); break;
      case (unsigned) MBBalanceTypes::ZMP_CONTROL: 
        bm = new ZmpControl<Scalar>(motionModule, cfg); break;
      default: bm = new MPComControl<Scalar>(motionModule, cfg); break;
  }
  return boost::shared_ptr<BalanceModule<Scalar> >(bm);
}

template class BalanceModule<MType>;
