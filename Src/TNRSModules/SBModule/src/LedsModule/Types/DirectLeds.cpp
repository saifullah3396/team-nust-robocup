/**
 * @file SBModule/src/LedsModule/Types/DirectLeds.cpp
 *
 * This file implements the class DirectLeds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jul 2018
 */

#include "SBModule/include/LedsModule/Types/DirectLeds.h"

void
DirectLeds::initiate()
{
  PRINT("DirectLeds.initiate()")
  inToReach = getBehaviorCast()->inToReach;
  inBehavior = true;
}

void
DirectLeds::update() // called once
{
  PRINT("DirectLeds.update()")
  vector<float> outLeds = ledRequest->getValue();
  for (int i = 0; i < NUM_LED_ACTUATORS; ++i) {
    //cout << "inToReach[" << i << "]: " << inToReach[i] << endl;
    if (inToReach[i] != inToReach[i]) continue; // NAN
    outLeds[i] = inToReach[i];
    //cout << "outLeds[" << i << "]: " << outLeds[i] << endl;
  }
  ledRequest->setValue(outLeds);
  BaseModule::publishModuleRequest(ledRequest);
  finish();
}

void
DirectLeds::finish()
{
  inBehavior = false;
}
