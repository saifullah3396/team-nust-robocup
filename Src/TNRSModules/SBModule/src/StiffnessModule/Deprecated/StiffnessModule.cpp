/**
 * @file SBModule/StiffnessModule/StiffnessModule.h
 *
 * This file implements the class for sending the stiffness changes 
 * requests to the control module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017 
 */

#include "StiffnessModule.h"

void
StiffnessModule::initiate()
{
  sToReach =
    (boost::static_pointer_cast < SBStiffnessConfig > (behaviorConfig))->timeToReachS;
  timeToReachS =
    (boost::static_pointer_cast < SBStiffnessConfig > (behaviorConfig))->timeToReachS;
  stiffnessesI = vector<float>(NUM_JOINTS, NAN);
  stiffnessesDelta = vector<float>(NUM_JOINTS, NAN);
  vector<float> stiffnessesF = vector<float>(NUM_JOINTS, sToReach);
  timeStep = 0.f;
  switch (behaviorConfig->type) {
  case SB_STIFFNESS_BODY:
    start = 0;
    end = NUM_JOINTS;
    break;
  case SB_STIFFNESS_HEAD:
    start = HEAD_END - HEAD_SIZE;
    end = HEAD_END;
    break;
  case SB_STIFFNESS_LARM:
    start = L_ARM_END - L_ARM_SIZE;
    end = L_ARM_END;
    break;
  case SB_STIFFNESS_RARM:
    start = R_ARM_END - R_ARM_SIZE;
    end = R_ARM_END;
    break;
  case SB_STIFFNESS_LLEG:
    start = L_LEG_END - L_LEG_SIZE;
    end = L_LEG_END;
    break;
  case SB_STIFFNESS_RLEG:
    start = R_LEG_END - R_LEG_SIZE;
    end = R_LEG_END;
    break;
  case SB_STIFFNESS_ROBOCUP:
    start = 0;
    end = NUM_JOINTS;
    stiffnessesF =
      vector<float>(
        sRobocupGameplay[0],
        sRobocupGameplay[0] + sizeof(sRobocupGameplay[0]) / sizeof(sRobocupGameplay[0][0]));
    break;
  }
  for (int i = start; i < end; ++i) {
    stiffnessesI[i] = IVAR(vector<float>, SBModule::jointStiffnessSensors)[i];
    float diff = stiffnessesF[i] - stiffnessesI[i];
    stiffnessesDelta[i] = diff != 0.f ? diff : stiffnessesDelta[i];
  }
  for (int i = start; i < end; ++i) {
    if (stiffnessesDelta[i] != NAN) {
      inBehavior = true;
      return;
    }
  }
  timeStep = timeStep + cycleTime;
  inBehavior = false;
}

void
StiffnessModule::update()
{
  if (!inBehavior) return;
  vector<float> outStiffnesses = OVAR(
    StiffnessRequest,
    SBModule::stiffnessRequest).getValue();
  if (timeStep > timeToReachS + cycleTime / 2) {
    inBehavior = false;
    return;
  } else {
    for (int i = start; i < end; ++i) {
      if (stiffnessesDelta[i] == 0.f) continue;
      outStiffnesses[i] =
        stiffnessesI[i] + stiffnessesDelta[i] * timeStep / timeToReachS;
    }
    OVAR(StiffnessRequest, SBModule::stiffnessRequest).setValue(outStiffnesses);
    OVAR(StiffnessRequest, SBModule::stiffnessRequest).id =
      sBModule->getModuleTime() * 100;
    timeStep = timeStep + cycleTime;
    cout << "jointrequestid: " << OVAR(
      StiffnessRequest,
      SBModule::stiffnessRequest).id << endl;
  }
}

void
StiffnessModule::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<SBStiffnessConfig>();
}
