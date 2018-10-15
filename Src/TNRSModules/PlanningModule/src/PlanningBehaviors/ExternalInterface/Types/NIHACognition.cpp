/**
 * @file PlanningBehaviors/CognitionModule/Types/NIHACognition.cpp
 *
 * This file implements the class NIHACognition
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "CommModule/include/CommRequest.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "SBModule/include/SBConfigs.h"
#include "PlanningModule/include/PlanningBehaviors/ExternalInterface/Types/NIHACognition.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/VisionUtils.h"

NIHACognitionConfigPtr NIHACognition::getBehaviorCast()
{
  return boost::static_pointer_cast <NIHACognitionConfig> (config);
}

void
NIHACognition::initiate()
{
  PRINT("NIHACognition.initiate()...")
  auto vRequest = boost::make_shared<SwitchVision>(true);
  BaseModule::publishModuleRequest(vRequest);
  inBehavior = true;
}

void
NIHACognition::update()
{
  //PRINT("NIHACognition.update()...")
	if (requestInProgress()) return;
  updatePostureAndStiffness();
  if (behaviorState == waitForConn) {
    waitForConnAction();
  } else if (behaviorState == sendHeader) {
    sendHeaderAction();
  } else if (behaviorState == sendData) {
    sendDataAction();
  }
}

void NIHACognition::finish()
{
  inBehavior = false;
}

void NIHACognition::waitForConnAction()
{
  if (!IVAR(vector<ClientInfo>, PlanningModule::clientsInfo).empty()) {
    behaviorState = sendHeader;
  }
}

void NIHACognition::sendHeaderAction()
{
  // Creating custom header
  string header = "";
  header += "{";
  header += "jointPositionSensors,";
  header += "jointPositionSensors,";
  header += "jointStiffnessSensors,";
  header += "jointTemperatureSensors,";
  header += "jointCurrentSensors,";
  header += "touchSensors,";
  header += "switchSensors,";
  header += "batterySensors,";
  header += "inertialSensors,";
  header += "sonarSensors,";
  header += "fsrSensors,";
  header += "ledSensors";
  header += "}";
  CommMessage cMsg(header, CommMsgTypes::NIHA_COGNITION_HEADER);
  CommRequestPtr request = boost::make_shared<SendMsgRequest>(cMsg);
  BaseModule::publishModuleRequest(request);
  behaviorState = sendData;
}

void NIHACognition::sendDataAction()
{
  if (IVAR(vector<ClientInfo>, PlanningModule::clientsInfo).empty()) {
    behaviorState = waitForConn;
  } else {
    string data;
    dataToString(data);
    CommMessage cMsg(data, CommMsgTypes::NIHA_COGNITION_DATA);
    CommRequestPtr request = boost::make_shared<SendMsgRequest>(cMsg);
    BaseModule::publishModuleRequest(request);
  }
}

void NIHACognition::dataToString(string& data)
{
  data = "";
  const size_t size = 12; // Sending 12 variables
  size_t commaLimit = size - 1;
  data += "{";
  data += DataUtils::varToString(size);
  data += ':';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::jointPositionSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::jointStiffnessSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::jointTemperatureSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::jointCurrentSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::touchSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::switchSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::batterySensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::inertialSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::sonarSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::fsrSensors));
  data += ',';
  data += DataUtils::varToString(IVAR(vector<float>, PlanningModule::ledSensors));
  data += "}";
}
