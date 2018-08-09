/**
 * @file MotionModule/BalanceZmpRefGen/BalanceZmpRefGen.h
 *
 * This file implements the class BalanceZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/BalanceZmpRefGen.h"

void BalanceZmpRefGen::initiate() {
  //! Static balance means Zmp = Com
  kM->getComWrtBase(refFrame, FEET_BASE, initZmpPosition);
}

void BalanceZmpRefGen::update() {
  for (unsigned i = 0; i <= nReferences; ++i) {
    if (timeStep + (i + 1) * cycleTime >= totalTime / 2) { // Change zmp after half the total time is passed
      zmpRef.xy[0][i] = targetZmp[0];
      zmpRef.xy[1][i] = targetZmp[1];
    } else {
      zmpRef.xy[0][i] = initZmpPosition[0];
      zmpRef.xy[1][i] = initZmpPosition[1];
    }
  }
  timeStep += cycleTime;
}
