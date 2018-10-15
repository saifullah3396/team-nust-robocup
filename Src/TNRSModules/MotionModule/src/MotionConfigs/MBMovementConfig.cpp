/**
 * @file MotionModule/src/MotionConfigs/MBMovementConfig.cpp
 *
 * This file implements the structs MBMovementConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
#include "MotionModule/include/MotionConfigs/MBMovementConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "Utils/include/RobotStateDefinitions.h"


MBMovementConfig::MBMovementConfig(
  const RobotPose2D<float>& goal,
  const bool& reachClosest,
  const MBMovementTypes& type) :
  MBConfig(MBIds::MOVEMENT, 360.f, (int)type),
  goal(goal), 
  reachClosest(reachClosest)
{
}

MBMovementConfig::MBMovementConfig(
  const RobotPose2D<float>& goal,
  const bool& reachClosest,
  const MBMovementTypes& type, 
  const MBHeadControlConfigPtr& htConfig) :
  MBConfig(MBIds::MOVEMENT, 360.f, (int)type),
  goal(goal), 
  reachClosest(reachClosest),
  htConfig(htConfig)
{
}

MBMovementConfig::MBMovementConfig(
  const MBMovementTypes& type, 
  const RobotPose2D<float>& goal,
  const bool& reachClosest,
  const MBPostureConfigPtr& postureConfig) :
  MBConfig(MBIds::MOVEMENT, 360.f, (int)type),
  goal(goal), 
  reachClosest(reachClosest),
  postureConfig(postureConfig)
{
}

MBMovementConfig::MBMovementConfig(
  const MBMovementTypes& type, 
  const RobotPose2D<float>& goal,
  const bool& reachClosest,
  const MBHeadControlConfigPtr& htConfig,
  const MBPostureConfigPtr& postureConfig) :
  MBConfig(MBIds::MOVEMENT, 360.f, (int)type),
  goal(goal), 
  reachClosest(reachClosest),
  htConfig(htConfig), 
  postureConfig(postureConfig)
{
}

void MBMovementConfig::validate() throw (BConfigException) 
{
}

bool MBMovementConfig::assignFromJson(const Json::Value& obj)
{
  BehaviorConfig::assignFromJson(obj);
  try {
    reachClosest = obj.get("reachClosest", false).asBool();
    goal.x = obj["goal"][0].asFloat();
    goal.y = obj["goal"][1].asFloat();
    goal.theta = obj["goal"][2].asFloat();
    const Json::Value& pcfgObj = obj["postureConfig"];
    if (!pcfgObj.isNull()) {
      auto pcfg = MBPostureConfig::makeFromJson(pcfgObj);
      if(pcfg->assignFromJson(pcfgObj)) {
        postureConfig = pcfg;
      }
    }
    
    const Json::Value& hcfgObj = obj["htConfig"];
    if (!hcfgObj.isNull()) {
      auto hccfg = MBHeadControlConfig::makeFromJson(hcfgObj);
      if(hccfg->assignFromJson(hcfgObj)) {
        htConfig = hccfg;
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << EnumToString(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    cout << e.what();
    return false;
  }
  return true;
}

Json::Value MBMovementConfig::getJson() 
{
  Json::Value obj = BehaviorConfig::getJson();
  try {
    obj["name"] = "MBMovement";
    obj["reachClosest"] = reachClosest;
    obj["goal"].append(goal.x);
    obj["goal"].append(goal.y);
    obj["goal"].append(goal.theta);
    if (postureConfig)
      obj["postureConfig"] = postureConfig->getJson();
    if (htConfig)
      obj["htConfig"] = htConfig->getJson();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << EnumToString(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    cout << e.what();
  }
  return obj;
}

boost::shared_ptr<MBMovementConfig> 
  MBMovementConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBMovementConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::MOVEMENT &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBMovementTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBMovementTypes::GO_TO_TARGET:
          config = 
            boost::make_shared<MBMovementConfig>(); break;
        
      }
    if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a movement behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}
