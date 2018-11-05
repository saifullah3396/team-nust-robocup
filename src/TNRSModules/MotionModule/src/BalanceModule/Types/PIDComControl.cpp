/**
 * @file MotionModule/BalanceModule/Types/PIDComControl.cpp
 *
 * This file implements the class PIDComControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/Types/PIDComControl.h"

template<typename Scalar>
PIDComControlConfigPtr PIDComControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <PIDComControlConfig> (this->config);
}

template<typename Scalar>
void
PIDComControl<Scalar>::initiate()
{
  // Behavior not yet defined
  ERROR("Behavior PIDComControl is undefined")
  return;
  #ifdef LOG_DATA
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("PIDComControl/Com.txt")).c_str(), 
    std::ofstream::out | std::ofstream::trunc
  );
  comLog.close();
  #endif
  supportLeg = getBehaviorCast()->supportLeg;
}

template<typename Scalar>
void
PIDComControl<Scalar>::update()
{
}

template<typename Scalar>
void
PIDComControl<Scalar>::finish()
{
  this->inBehavior = false;
}

template class PIDComControl<MType>;
