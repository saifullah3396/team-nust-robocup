/**
 * @file PlanningBehaviors/ExternalInterface/Types/UserRequestsHandler.cpp
 *
 * This file implements the class UserRequestsHandler
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "CommModule/include/CommRequest.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "SBModule/include/SBConfigs.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/UserRequestsHandler.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/VisionUtils.h"

UserRequestsHandlerConfigPtr UserRequestsHandler::getBehaviorCast()
{
  return boost::static_pointer_cast <UserRequestsHandlerConfig> (config);
}

void
UserRequestsHandler::initiate()
{
  PRINT("UserRequestsHandler.initiate()...")
  inBehavior = true;
}

void
UserRequestsHandler::update()
{
  //PRINT("UserRequestsHandler.update()...")
	if (requestInProgress()) return;
  updatePostureAndStiffness();
  if (behaviorState == waitForConn) {
    waitForConnAction();
  } else if (behaviorState == onRequest) {
    onRequestAction();
  }
}

void UserRequestsHandler::finish()
{
  inBehavior = false;
}

void UserRequestsHandler::waitForConnAction()
{
  if (!IVAR(vector<ClientInfo>, PlanningModule::clientsInfo).empty()) {
    behaviorState = onRequest;
  }
}

void UserRequestsHandler::onRequestAction()
{
  cout << "onRequest" << endl;
}
