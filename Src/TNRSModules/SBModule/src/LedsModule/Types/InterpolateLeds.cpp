/**
 * @file SBModule/src/LedsModule/Types/InterpolateLeds.cpp
 *
 * This file implements the class InterpolateLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "SBModule/include/LedsModule/Types/InterpolateLeds.h"

void
InterpolateLeds::initiate()
{
  PRINT("InterpolateLeds.initiate()")
  inToReach = getBehaviorCast()->inToReach;
  timeToReachIn = getBehaviorCast()->timeToReachIn;
  ledsI = IVAR(vector<float>, SBModule::ledSensors);
  ledsDelta = vector<float>(NUM_LED_ACTUATORS, NAN);

  for (int i = 0; i < NUM_LED_ACTUATORS; ++i) {
    if (inToReach[i] != inToReach[i]) continue; // NAN
    ledsDelta[i] = inToReach[i] - ledsI[i];
  }
  inBehavior = true;
}

void
InterpolateLeds::update()
{
  PRINT("InterpolateLeds.update()")
  if (execTime > timeToReachIn + cycleTime / 2) {
    finish();
  } else {
    //cout << "runTime: " << runTime << endl;
    vector<float> outLeds = ledRequest->getValue();
    for (int i = 0; i < NUM_LED_ACTUATORS; ++i) {
      if (inToReach[i] != inToReach[i]) continue; // NAN
      outLeds[i] = ledsI[i] + ledsDelta[i] * runTime / timeToReachIn;
      //cout << "outLeds[" << i << "]: " << outLeds[i] << endl;
    }
    ledRequest->setValue(outLeds);
    BaseModule::publishModuleRequest(ledRequest);
    execTime += cycleTime;
  }
}

void
InterpolateLeds::finish()
{
  inBehavior = false;
}
