/**
 * @file MotionModule/BalanceZmpRefGen/BalanceZmpRefGen.h
 *
 * This file implements the class BalanceZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/BalanceZmpRefGen.h"

template<typename Scalar>
void BalanceZmpRefGen<Scalar>::initiate() {
  //! Static balance means Zmp = Com
  this->kM->computeComWrtBase(this->refFrame, FEET_BASE, initZmpPosition);
}

template<typename Scalar>
void BalanceZmpRefGen<Scalar>::update() {
  for (unsigned i = 0; i <= this->nReferences; ++i) {
    if (timeStep + (i + 1) * this->cycleTime >= totalTime / 2) { // Change zmp after 1/5 the total time is passed
      this->zmpRef.xy[0][i] = targetZmp[0];
      this->zmpRef.xy[1][i] = targetZmp[1];
    } else {
      this->zmpRef.xy[0][i] = initZmpPosition[0];
      this->zmpRef.xy[1][i] = initZmpPosition[1];
    }
  }
  timeStep += this->cycleTime;
}

template class BalanceZmpRefGen<MType>;
